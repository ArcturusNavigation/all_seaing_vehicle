#ifndef ALL_SEAING_COMMON__STATUS_REPORTER_HPP
#define ALL_SEAING_COMMON__STATUS_REPORTER_HPP

// Live per-component status for C++ ROS 2 nodes (issue #480). C++ counterpart of
// all_seaing_common/status_reporter.py.
//
// A node records what its pipeline is doing as named counters (e.g. "boxes_in",
// "rejected/no_points"), gauges (latest value, e.g. "tracked_obstacles") and timings
// (e.g. "proc_ms"). A background thread (independent of the node's executor, so a slow
// callback doesn't delay it) publishes them every `status_period` seconds on /diagnostics
// as one diagnostic_msgs/DiagnosticStatus named "<node>[/<component>]", viewable in
// Foxglove's Diagnostics panel or `ros2 topic echo /diagnostics`. For each period it reports:
//   counters: "<name>" = total since start, "<name>/rate" = per second over the last period
//   gauges:   "<name>" = latest value
//   timings:  "<name>/mean", "<name>/max", "<name>/count" over the last period
//   "period_s", "uptime_s" (since the reporter started: total / uptime_s = average rate), plus a level (OK / WARN / ERROR / STALE) and a message
// The level is STALE when the heartbeat counter (default "updates") didn't change during a
// period (the pipeline isn't running), WARN/ERROR when set with set_level(), else OK.
//
// Recording calls only lock a mutex briefly and never do I/O. This is independent of
// RunLogger (#501): use RunLogger for per-event detail, StatusReporter for live aggregates.
//
// Usage:
//   m_status = std::make_unique<all_seaing_common::StatusReporter>(this);  // or (this, "front")
//   m_status->count("updates");                 // once per pipeline iteration (heartbeat)
//   m_status->count("rejected/no_points");
//   m_status->gauge("boxes_in_frame", n);
//   m_status->timing("proc_ms", ms);
//   auto t = m_status->timer("proc_ms");        // RAII: records elapsed ms when it goes out of scope
//
// ROS parameters (declared on the node): status_enabled (bool, true),
// status_period (double, 1.0 s), status_heartbeat (string, "updates").

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <condition_variable>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace all_seaing_common {

class StatusReporter {
public:
    // component: optional suffix of the status name, e.g. the camera name
    explicit StatusReporter(rclcpp::Node *node, const std::string &component = "");
    ~StatusReporter();

    StatusReporter(const StatusReporter &) = delete;
    StatusReporter &operator=(const StatusReporter &) = delete;

    void count(const std::string &name, uint64_t n = 1);
    void gauge(const std::string &name, double value);
    void gauge(const std::string &name, const std::string &value);
    void timing(const std::string &name, double ms);
    // Level/message shown until changed (DiagnosticStatus::OK/WARN/ERROR)
    void set_level(uint8_t level, const std::string &message);

    // RAII timer: records the elapsed time [ms] under `name` when destroyed
    class ScopedTimer {
    public:
        ScopedTimer(StatusReporter *r, std::string name)
            : m_r(r), m_name(std::move(name)), m_start(std::chrono::steady_clock::now()) {}
        ~ScopedTimer() {
            if (m_r) {
                m_r->timing(m_name, std::chrono::duration<double, std::milli>(
                                        std::chrono::steady_clock::now() - m_start).count());
            }
        }
        ScopedTimer(ScopedTimer &&o) noexcept : m_r(o.m_r), m_name(std::move(o.m_name)), m_start(o.m_start) { o.m_r = nullptr; }
        ScopedTimer(const ScopedTimer &) = delete;
    private:
        StatusReporter *m_r;
        std::string m_name;
        std::chrono::steady_clock::time_point m_start;
    };
    ScopedTimer timer(const std::string &name) { return ScopedTimer(this, name); }

private:
    struct Timing {
        double sum{0}, max{0};
        uint64_t n{0};
    };

    void run();
    diagnostic_msgs::msg::DiagnosticArray build();

    bool m_enabled{true};
    double m_period{1.0};
    std::string m_name, m_hardware_id, m_heartbeat;
    rclcpp::Clock::SharedPtr m_clock;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_pub;

    std::mutex m_mutex;
    std::map<std::string, uint64_t> m_counters, m_counters_prev;
    std::map<std::string, std::string> m_gauges;
    std::map<std::string, Timing> m_timings;
    uint8_t m_level{diagnostic_msgs::msg::DiagnosticStatus::OK};
    std::string m_message{"OK"};
    std::chrono::steady_clock::time_point m_last_publish, m_start;

    std::mutex m_stop_mutex;
    std::condition_variable m_stop_cv;
    bool m_stop{false};
    std::thread m_thread;
};

}  // namespace all_seaing_common

#endif  // ALL_SEAING_COMMON__STATUS_REPORTER_HPP
