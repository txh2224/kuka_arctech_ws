#ifndef UTILS_DEBUG_RECORDER_H
#define UTILS_DEBUG_RECORDER_H

#include <filesystem>
#include <memory>
#include <string>

#include <atomic_adapter_interfaces/action/execute_backend_job.hpp>
#include <atomic_adapter_interfaces/msg/backend_status.hpp>
#include <atomic_adapter_interfaces/srv/stop_backend_job.hpp>
#include <rclcpp_action/types.hpp>

namespace utils {

// 每个节点实例独立记录一份 JSONL，后台线程负责写入和刷新。
class DebugRecorder {
    using ExecuteBackendJob = atomic_adapter_interfaces::action::ExecuteBackendJob;
    using BackendStatus = atomic_adapter_interfaces::msg::BackendStatus;
    using StopBackendJob = atomic_adapter_interfaces::srv::StopBackendJob;

public:
    DebugRecorder(const std::string &directory, const std::string &node_name);
    ~DebugRecorder();

    const std::filesystem::path &path() const noexcept;

    void record_goal(
        const rclcpp_action::GoalUUID &uuid,
        const ExecuteBackendJob::Goal &goal,
        bool accepted,
        const char *reason) noexcept;
    void record_cancel(
        const rclcpp_action::GoalUUID &uuid,
        const ExecuteBackendJob::Goal &goal,
        bool accepted) noexcept;
    void record_feedback(
        const rclcpp_action::GoalUUID &uuid,
        const ExecuteBackendJob::Goal &goal,
        const ExecuteBackendJob::Feedback &feedback) noexcept;
    void record_result(
        const rclcpp_action::GoalUUID &uuid,
        const ExecuteBackendJob::Goal &goal,
        const ExecuteBackendJob::Result &result,
        const char *action_state) noexcept;
    void record_stop_request(const StopBackendJob::Request &request) noexcept;
    void record_stop_response(
        const StopBackendJob::Request &request,
        const StopBackendJob::Response &response) noexcept;
    void record_status(const BackendStatus &status) noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace utils

#endif  // UTILS_DEBUG_RECORDER_H
