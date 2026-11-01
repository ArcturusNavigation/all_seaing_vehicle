#include "all_seaing_common/run_logger.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <fcntl.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace all_seaing_common {

namespace {

const char *LEVEL_NAMES[] = {"DEBUG", "INFO", "WARN", "ERROR"};

std::string env_or(const char *name, const std::string &fallback) {
    const char *v = std::getenv(name);
    return (v && *v) ? std::string(v) : fallback;
}

std::string expand_home(const std::string &path) {
    if (!path.empty() && path[0] == '~') {
        return env_or("HOME", "") + path.substr(1);
    }
    return path;
}

std::string log_dir() { return expand_home(env_or("ARCTURUS_LOG_DIR", "~/arcturus_logs")); }
std::string buffer_dir() { return expand_home(env_or("ARCTURUS_LOG_BUFFER_DIR", "/dev/shm/arcturus_logs")); }

// Same logic as run_logger.current_run_id(): env var, else the shared marker file (created
// atomically by the first logger of the run)
std::string current_run_id() {
    std::string env_id = env_or("ARCTURUS_RUN_ID", "");
    if (!env_id.empty()) return env_id;
    fs::create_directories(buffer_dir());
    std::string marker = buffer_dir() + "/current_run";
    std::time_t now = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    std::string new_id(buf);
    int fd = ::open(marker.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0644);
    if (fd >= 0) {
        ssize_t written = ::write(fd, new_id.data(), new_id.size());
        (void)written;
        ::close(fd);
        return new_id;
    }
    std::ifstream in(marker);
    std::string existing;
    std::getline(in, existing);
    return existing.empty() ? new_id : existing;
}

void append_json_string(std::string &out, const std::string &s) {
    out += '"';
    for (unsigned char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (c < 0x20) {
                char esc[8];
                std::snprintf(esc, sizeof(esc), "\\u%04x", c);
                out += esc;
            } else {
                out += static_cast<char>(c);
            }
        }
    }
    out += '"';
}

void append_json_number(std::string &out, double v) {
    if (!std::isfinite(v)) {
        out += "null";  // JSON has no NaN/Inf
        return;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    out += buf;
}

double wall_now() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// [x, y, ...]
void append_json_numbers(std::string &out, const std::vector<double> &v) {
    out += '[';
    for (size_t i = 0; i < v.size(); i++) {
        if (i) out += ", ";
        append_json_number(out, v[i]);
    }
    out += ']';
}

}  // namespace

void LogValue::append_json(std::string &out) const {
    std::visit(
        [&out](const auto &v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, bool>) {
                out += v ? "true" : "false";
            } else if constexpr (std::is_same_v<T, int64_t>) {
                out += std::to_string(v);
            } else if constexpr (std::is_same_v<T, double>) {
                append_json_number(out, v);
            } else if constexpr (std::is_same_v<T, std::string>) {
                append_json_string(out, v);
            } else if constexpr (std::is_same_v<T, std::vector<double>>) {
                append_json_numbers(out, v);
            } else {
                out += '[';
                for (size_t i = 0; i < v.size(); i++) {
                    if (i) out += ", ";
                    append_json_numbers(out, v[i]);
                }
                out += ']';
            }
        },
        m_value);
}

RunLogger::RunLogger(rclcpp::Node *node, const std::string &name) : m_node(node) {
    m_enabled = node->declare_parameter<bool>("run_log_enabled", true);
    std::string min_level = node->declare_parameter<std::string>("run_log_min_level", "DEBUG");
    m_queue_size = static_cast<size_t>(node->declare_parameter<int>("run_log_queue_size", 10000));
    m_persist_period = node->declare_parameter<double>("run_log_persist_period", 2.0);
    m_max_buffer_bytes =
        static_cast<uint64_t>(node->declare_parameter<double>("run_log_max_buffer_mb", 64.0) * 1e6);
    bool publish = node->declare_parameter<bool>("run_log_publish", true);
    std::string pub_level = node->declare_parameter<std::string>("run_log_publish_min_level", "DEBUG");
    m_publish_period = node->declare_parameter<double>("run_log_publish_period", 0.0);
    std::string topic = node->declare_parameter<std::string>("run_log_topic", "/run_log");
    for (int i = 0; i < 4; i++) {
        if (min_level == LEVEL_NAMES[i]) m_min_level = i;
        if (pub_level == LEVEL_NAMES[i]) m_pub_min_level = i;
    }
    if (!m_enabled) return;
    if (publish) m_pub = node->create_publisher<std_msgs::msg::String>(topic, 100);

    m_node_name = name;
    if (m_node_name.empty()) {
        m_node_name = node->get_fully_qualified_name();
        while (!m_node_name.empty() && m_node_name[0] == '/') m_node_name.erase(0, 1);
        for (auto &c : m_node_name) {
            if (c == '/') c = '.';
        }
    }

    try {
        m_run_id = current_run_id();
        fs::path run_buf = fs::path(buffer_dir()) / m_run_id;
        fs::create_directories(run_buf);
        m_buffer_path = (run_buf / (m_node_name + "." + std::to_string(::getpid()) + ".jsonl")).string();
        m_persist_path = (fs::path(log_dir()) / m_run_id / (m_node_name + ".jsonl")).string();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "RunLogger disabled, cannot set up log directories: %s",
                     e.what());
        m_enabled = false;
        return;
    }

    m_thread = std::thread(&RunLogger::run, this);
    info("run_logger_started",
         {{"run_id", m_run_id}, {"pid", static_cast<int64_t>(::getpid())}, {"file", m_persist_path}});
}

RunLogger::~RunLogger() {
    if (!m_thread.joinable()) return;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop = true;
    }
    m_cv.notify_one();
    m_thread.join();
}

void RunLogger::log(Level level, const std::string &event, LogFields fields) {
    if (!m_enabled || static_cast<int>(level) < m_min_level) return;
    double t = 0.0;
    try {
        t = m_node->get_clock()->now().seconds();
    } catch (...) {
    }
    try {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_stop) return;
        if (m_queue.size() >= m_queue_size) {
            m_queue.pop_front();
            m_dropped_queue++;
        }
        m_queue.push_back(Entry{t, wall_now(), level, event, std::move(fields)});
    } catch (...) {
        // bad_alloc etc.: never propagate into the node
    }
    if (m_pub && m_publish_period <= 0.0) m_cv.notify_one();  // real time: publish now
}

void RunLogger::flush() {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_flush_requested = true;
    }
    m_cv.notify_one();
}

void RunLogger::run() {
    using clock = std::chrono::steady_clock;
    auto period = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(m_persist_period));
    // wake at the publishing period (writes to the RAM buffer are cheap), persist less often
    // (publish period 0: woken by each log() call, the period only bounds the persist latency)
    auto wake_period = m_pub && m_publish_period > 0.0
                           ? std::chrono::duration_cast<clock::duration>(
                                 std::chrono::duration<double>(std::min(m_persist_period, m_publish_period)))
                           : period;
    const bool immediate = m_pub && m_publish_period <= 0.0;
    auto last_persist = clock::now();
    while (true) {
        std::deque<Entry> batch;
        bool stopping, flush_now;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait_for(lock, wake_period,
                          [this, immediate] { return m_stop || m_flush_requested || (immediate && !m_queue.empty()); });
            stopping = m_stop;
            flush_now = m_flush_requested;
            m_flush_requested = false;
            batch.swap(m_queue);
        }
        bool persist_now = stopping || flush_now || clock::now() - last_persist >= period;
        if (persist_now) report_drops(batch);
        write_queued(batch);
        if (persist_now) {
            persist();
            last_persist = clock::now();
        }
        if (stopping) break;
    }
    std::error_code ec;
    if (fs::exists(m_buffer_path, ec) && fs::file_size(m_buffer_path, ec) == 0) {
        fs::remove(m_buffer_path, ec);  // fully persisted; otherwise leave it for recovery
    }
}

void RunLogger::report_drops(std::deque<Entry> &batch) {
    uint64_t dq = m_dropped_queue.exchange(0);
    if (dq == 0 && m_dropped_io == 0) return;
    batch.push_back(Entry{NAN, wall_now(), Level::WARN, "run_logger_dropped",  // NAN -> "t": null, as in Python
                          {{"dropped_queue", static_cast<int64_t>(dq)},
                           {"dropped_io", static_cast<int64_t>(m_dropped_io)},
                           {"io_errors", static_cast<int64_t>(m_io_errors)}}});
    m_dropped_io = 0;
}

std::string RunLogger::serialize(const Entry &e) const {
    std::string data;
    {
        data += "{\"t\": ";
        append_json_number(data, e.t);
        data += ", \"wall\": ";
        append_json_number(data, e.wall);
        data += ", \"node\": ";
        append_json_string(data, m_node_name);
        data += ", \"level\": \"";
        data += LEVEL_NAMES[static_cast<int>(e.level)];
        data += "\", \"event\": ";
        append_json_string(data, e.event);
        for (const auto &[key, value] : e.fields) {
            data += ", ";
            append_json_string(data, key);
            data += ": ";
            value.append_json(data);
        }
        data += "}";
    }
    return data;
}

void RunLogger::write_queued(std::deque<Entry> &batch) {
    if (batch.empty()) return;
    std::string data, published;
    for (const auto &e : batch) {
        std::string line = serialize(e);
        if (m_pub && static_cast<int>(e.level) >= m_pub_min_level) {
            if (!published.empty()) published += '\n';
            published += line;
        }
        data += line;
        data += '\n';
    }
    if (m_pub && !published.empty()) {
        try {
            std_msgs::msg::String msg;
            msg.data = std::move(published);
            m_pub->publish(msg);
        } catch (...) {
            // context shut down, etc.: the file log is unaffected
        }
    }
    std::error_code ec;
    uint64_t current = fs::exists(m_buffer_path, ec) ? fs::file_size(m_buffer_path, ec) : 0;
    if (current + data.size() > m_max_buffer_bytes) {
        m_dropped_io += batch.size();
        return;
    }
    std::ofstream out(m_buffer_path, std::ios::binary | std::ios::app);
    out << data;
    if (!out) {
        m_io_errors++;
        m_dropped_io += batch.size();
    }
}

void RunLogger::persist() {
    std::error_code ec;
    if (!fs::exists(m_buffer_path, ec) || fs::file_size(m_buffer_path, ec) == 0) return;
    try {
        fs::create_directories(fs::path(m_persist_path).parent_path());
        {
            std::ifstream in(m_buffer_path, std::ios::binary);
            std::ofstream out(m_persist_path, std::ios::binary | std::ios::app);
            out << in.rdbuf();
            out.flush();
            if (!out) throw std::runtime_error("write failed");
        }
        std::ofstream truncate(m_buffer_path, std::ios::binary | std::ios::trunc);  // keep only the unpersisted tail
    } catch (...) {
        m_io_errors++;  // keep the buffer, retry next period
    }
}

}  // namespace all_seaing_common
