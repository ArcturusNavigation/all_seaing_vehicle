#ifndef ALL_SEAING_COMMON__RUN_LOGGER_HPP
#define ALL_SEAING_COMMON__RUN_LOGGER_HPP

// Per-run structured text logger for C++ ROS 2 nodes (issue #501). C++ counterpart of
// all_seaing_common/run_logger.py: same files, run grouping, entry format and guarantees.
// See that module's docstring for the full design. In short:
//   - log() only pushes onto a bounded in-memory queue (drops the oldest when full). It
//     never blocks on I/O and never throws.
//   - A background thread serializes entries to JSON lines, appends them to a RAM buffer
//     file (<buffer_dir>/<run_id>/<node>.<pid>.jsonl, default buffer_dir /dev/shm/arcturus_logs),
//     and every `run_log_persist_period` seconds (and on destruction) appends the buffer
//     to <log_dir>/<run_id>/<node>.jsonl (default log_dir ~/arcturus_logs) and clears it.
//   - Buffers of killed nodes stay in /dev/shm and are recovered by the next Python
//     logger start or `python3 -m all_seaing_common.run_logger --recover`.
//   - Entries are also published (batched, from the background thread) as JSON lines on the
//     `/run_log` topic (std_msgs/String) for live analysis (e.g. control panel via foxglove_bridge).
//   - Environment: ARCTURUS_LOG_DIR, ARCTURUS_LOG_BUFFER_DIR, ARCTURUS_RUN_ID.
//
// Usage:
//   #include "all_seaing_common/run_logger.hpp"
//   m_run_log = std::make_unique<all_seaing_common::RunLogger>(this);
//   m_run_log->info("fusion", {{"camera", "front"}, {"n_boxes", 3}, {"proc_ms", 12.3}});
//   m_run_log->warn("detection_rejected", {{"label", label}, {"reason", "hsv_mismatch"}});
//
// ROS parameters (declared on the node): run_log_enabled (bool, true),
// run_log_min_level (string, "DEBUG"), run_log_queue_size (int, 10000),
// run_log_persist_period (double, 2.0 s), run_log_max_buffer_mb (double, 64.0),
// run_log_publish (bool, true), run_log_publish_min_level (string, "DEBUG"),
// run_log_publish_period (double, 0 s: publish each entry as soon as it is logged; > 0: batch over that
// period), run_log_topic (string, "/run_log").

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace all_seaing_common {

// A JSON-serializable field value: bool, integer, floating point, string, a list of numbers, or a
// list of lists of numbers (e.g. polygons, point lists)
class LogValue {
public:
    using Variant =
        std::variant<bool, int64_t, double, std::string, std::vector<double>, std::vector<std::vector<double>>>;

    LogValue(bool v) : m_value(v) {}
    LogValue(int v) : m_value(static_cast<int64_t>(v)) {}
    LogValue(long v) : m_value(static_cast<int64_t>(v)) {}
    LogValue(long long v) : m_value(static_cast<int64_t>(v)) {}
    LogValue(unsigned v) : m_value(static_cast<int64_t>(v)) {}
    LogValue(unsigned long v) : m_value(static_cast<int64_t>(v)) {}
    LogValue(unsigned long long v) : m_value(static_cast<int64_t>(v)) {}
    LogValue(float v) : m_value(static_cast<double>(v)) {}
    LogValue(double v) : m_value(v) {}
    LogValue(const char *v) : m_value(std::string(v)) {}
    LogValue(std::string v) : m_value(std::move(v)) {}
    LogValue(std::vector<double> v) : m_value(std::move(v)) {}
    LogValue(std::initializer_list<double> v) : m_value(std::vector<double>(v)) {}
    LogValue(std::vector<std::vector<double>> v) : m_value(std::move(v)) {}

    // Append the JSON encoding of this value to out
    void append_json(std::string &out) const;

private:
    Variant m_value;
};

using LogFields = std::vector<std::pair<std::string, LogValue>>;

class RunLogger {
public:
    enum class Level { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3 };

    // node: the owning node (name, ROS clock, parameters). name: file name override
    // (default: fully qualified node name with '/' -> '.').
    explicit RunLogger(rclcpp::Node *node, const std::string &name = "");
    ~RunLogger();  // final write + persist, then joins the background thread

    RunLogger(const RunLogger &) = delete;
    RunLogger &operator=(const RunLogger &) = delete;

    // Queue one entry. Never blocks on I/O, never throws.
    void log(Level level, const std::string &event, LogFields fields = {});
    void debug(const std::string &event, LogFields fields = {}) { log(Level::DEBUG, event, std::move(fields)); }
    void info(const std::string &event, LogFields fields = {}) { log(Level::INFO, event, std::move(fields)); }
    void warn(const std::string &event, LogFields fields = {}) { log(Level::WARN, event, std::move(fields)); }
    void error(const std::string &event, LogFields fields = {}) { log(Level::ERROR, event, std::move(fields)); }

    void flush();  // ask the background thread to write + persist now (non-blocking)
    bool enabled() const { return m_enabled; }
    const std::string &run_id() const { return m_run_id; }

private:
    struct Entry {
        double t;     // ROS time [s]
        double wall;  // wall time [s]
        Level level;
        std::string event;
        LogFields fields;
    };

    void run();
    void write_queued(std::deque<Entry> &batch);
    std::string serialize(const Entry &e) const;
    void persist();
    void report_drops(std::deque<Entry> &batch);

    rclcpp::Node *m_node;
    bool m_enabled{false};
    int m_min_level{0};
    size_t m_queue_size{10000};
    double m_persist_period{2.0};
    uint64_t m_max_buffer_bytes{64000000};
    int m_pub_min_level{0};
    double m_publish_period{0.0};  // 0: publish immediately (no batching)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub;
    std::string m_node_name, m_run_id, m_buffer_path, m_persist_path;

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::deque<Entry> m_queue;
    bool m_stop{false};
    bool m_flush_requested{false};
    std::atomic<uint64_t> m_dropped_queue{0};
    uint64_t m_dropped_io{0};
    uint64_t m_io_errors{0};
    std::thread m_thread;
};

}  // namespace all_seaing_common

#endif  // ALL_SEAING_COMMON__RUN_LOGGER_HPP
