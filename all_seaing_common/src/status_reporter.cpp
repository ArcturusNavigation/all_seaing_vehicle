#include "all_seaing_common/status_reporter.hpp"

#include <cstdio>

namespace all_seaing_common {

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace {
std::string fmt(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.4g", v);
    return buf;
}
}  // namespace

StatusReporter::StatusReporter(rclcpp::Node *node, const std::string &component) {
    // Several reporters can share a node (e.g. one per camera or topic): declare once
    if (!node->has_parameter("status_enabled")) node->declare_parameter<bool>("status_enabled", true);
    if (!node->has_parameter("status_period")) node->declare_parameter<double>("status_period", 1.0);
    if (!node->has_parameter("status_heartbeat")) node->declare_parameter<std::string>("status_heartbeat", "updates");
    m_enabled = node->get_parameter("status_enabled").as_bool();
    m_period = node->get_parameter("status_period").as_double();
    m_heartbeat = node->get_parameter("status_heartbeat").as_string();
    if (!m_enabled) return;

    m_hardware_id = node->get_fully_qualified_name();
    m_name = m_hardware_id.substr(m_hardware_id.find_first_not_of('/'));
    if (!component.empty()) m_name += "/" + component;
    m_clock = node->get_clock();
    m_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    m_last_publish = m_start = std::chrono::steady_clock::now();
    m_thread = std::thread(&StatusReporter::run, this);
}

StatusReporter::~StatusReporter() {
    if (!m_thread.joinable()) return;
    {
        std::lock_guard<std::mutex> lock(m_stop_mutex);
        m_stop = true;
    }
    m_stop_cv.notify_one();
    m_thread.join();
}

void StatusReporter::count(const std::string &name, uint64_t n) {
    if (!m_enabled) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_counters[name] += n;
}

void StatusReporter::gauge(const std::string &name, double value) { gauge(name, fmt(value)); }

void StatusReporter::gauge(const std::string &name, const std::string &value) {
    if (!m_enabled) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_gauges[name] = value;
}

void StatusReporter::timing(const std::string &name, double ms) {
    if (!m_enabled) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    Timing &t = m_timings[name];
    t.sum += ms;
    t.max = std::max(t.max, ms);
    t.n++;
}

void StatusReporter::set_level(uint8_t level, const std::string &message) {
    if (!m_enabled) return;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_level = level;
    m_message = message;
}

diagnostic_msgs::msg::DiagnosticArray StatusReporter::build() {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - m_last_publish).count();
    m_last_publish = now;

    DiagnosticStatus status;
    status.name = m_name;
    status.hardware_id = m_hardware_id;
    status.values.push_back(KeyValue().set__key("period_s").set__value(fmt(dt)));
    status.values.push_back(KeyValue().set__key("uptime_s").set__value(
        fmt(std::chrono::duration<double>(now - m_start).count())));
    for (const auto &[name, total] : m_counters) {
        uint64_t prev = m_counters_prev.count(name) ? m_counters_prev[name] : 0;
        status.values.push_back(KeyValue().set__key(name).set__value(std::to_string(total)));
        status.values.push_back(KeyValue().set__key(name + "/rate").set__value(fmt(dt > 0 ? (total - prev) / dt : 0)));
    }
    for (const auto &[name, value] : m_gauges) {
        status.values.push_back(KeyValue().set__key(name).set__value(value));
    }
    for (auto &[name, t] : m_timings) {
        status.values.push_back(KeyValue().set__key(name + "/mean").set__value(fmt(t.n ? t.sum / t.n : 0)));
        status.values.push_back(KeyValue().set__key(name + "/max").set__value(fmt(t.max)));
        status.values.push_back(KeyValue().set__key(name + "/count").set__value(std::to_string(t.n)));
        t = Timing();  // timings are per period
    }

    uint64_t hb = m_counters.count(m_heartbeat) ? m_counters[m_heartbeat] : 0;
    uint64_t hb_prev = m_counters_prev.count(m_heartbeat) ? m_counters_prev[m_heartbeat] : 0;
    if (hb == hb_prev) {
        status.level = DiagnosticStatus::STALE;
        status.message = "no '" + m_heartbeat + "' in the last " + fmt(dt) + " s";
    } else {
        status.level = m_level;
        status.message = m_message;
    }
    m_counters_prev = m_counters;

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = m_clock->now();
    msg.status.push_back(status);
    return msg;
}

void StatusReporter::run() {
    auto period = std::chrono::duration<double>(m_period);
    while (true) {
        {
            std::unique_lock<std::mutex> lock(m_stop_mutex);
            if (m_stop_cv.wait_for(lock, period, [this] { return m_stop; })) return;
        }
        try {
            m_pub->publish(build());
        } catch (const std::exception &) {
            // context shut down mid-publish etc.: never take the node down
        }
    }
}

}  // namespace all_seaing_common
