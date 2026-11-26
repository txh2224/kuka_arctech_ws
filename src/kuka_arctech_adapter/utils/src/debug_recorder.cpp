#include "debug_recorder.h"

#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <unistd.h>

namespace utils {
namespace {

using Json = nlohmann::json;
using Clock = std::chrono::system_clock;
using ExecuteBackendJob = atomic_adapter_interfaces::action::ExecuteBackendJob;
using AtomicInstruction = atomic_adapter_interfaces::msg::AtomicInstruction;
using BackendStatus = atomic_adapter_interfaces::msg::BackendStatus;

constexpr std::size_t kMaxPendingEvents = 1024;

struct PendingEvent {
    Clock::time_point timestamp;
    const char *name;
    Json fields;
};

std::filesystem::path make_record_path(
    const std::string &directory,
    const std::string &node_name) {
    const auto root = std::filesystem::absolute(directory);
    std::filesystem::create_directories(root);

    std::string file_prefix = node_name.empty() ? "node" : node_name;
    for (char &ch : file_prefix) {
        if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '-' && ch != '_') {
            ch = '_';
        }
    }

    const auto time_id = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now().time_since_epoch()).count();
    static std::atomic<unsigned long> next_id{0};
    const auto id = next_id.fetch_add(1, std::memory_order_relaxed);
    return root / (file_prefix + "-" + std::to_string(time_id) + "-" +
                   std::to_string(getpid()) + "-" + std::to_string(id) + ".jsonl");
}

std::string format_timestamp(Clock::time_point timestamp) {
    const auto seconds = std::chrono::floor<std::chrono::seconds>(timestamp);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        timestamp - seconds).count();
    const auto time = Clock::to_time_t(seconds);
    std::tm utc{};
    gmtime_r(&time, &utc);

    std::ostringstream formatted;
    formatted << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S")
              << '.' << std::setfill('0') << std::setw(3) << millis << 'Z';
    return formatted.str();
}

Json action_identity(
    const rclcpp_action::GoalUUID &uuid,
    const ExecuteBackendJob::Goal &goal) {
    return Json{
        {"goal_uuid", rclcpp_action::to_string(uuid)},
        {"job_id", goal.job_id},
        {"trace_id", goal.trace_id},
    };
}

Json instruction_fields(const AtomicInstruction &instruction) {
    return Json{
        {"schema_version", instruction.schema_version},
        {"instruction_id", instruction.instruction_id},
        {"instance_id", instruction.instance_id},
        {"correlation_id", instruction.correlation_id},
        {"target_device", instruction.target_device},
        {"capability_id", instruction.capability_id},
        {"action", instruction.action},
        {"object_json", instruction.object_json},
        {"parameter_json", instruction.parameter_json},
        {"constraint_json", instruction.constraint_json},
        {"safety_json", instruction.safety_json},
        {"result_contract_json", instruction.result_contract_json},
        {"timeout_ms", instruction.timeout_ms},
        {"priority", instruction.priority},
        {"failure_policy", instruction.failure_policy},
    };
}

Json status_fields(const BackendStatus &status) {
    return Json{
        {"backend", status.backend},
        {"controller_connected", status.controller_connected},
        {"program_state", status.program_state},
        {"weld_state", status.weld_state},
        {"current_job_id", status.current_job_id},
        {"current_program", status.current_program},
        {"fault_code", status.fault_code},
        {"message", status.message},
    };
}

}  // namespace

class DebugRecorder::Impl {
public:
    Impl(const std::string &directory, const std::string &node_name)
        : path_(make_record_path(directory, node_name)),
          node_(node_name),
          file_(path_, std::ios::out | std::ios::app | std::ios::binary) {
        if (!file_) {
            throw std::runtime_error("无法打开调试记录文件 " + path_.string());
        }
        worker_ = std::thread(&Impl::write_loop, this);
    }

    ~Impl() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        ready_.notify_one();
        worker_.join();
    }

    const std::filesystem::path &path() const noexcept {
        return path_;
    }

    template<class MakeFields>
    void record(const char *event, MakeFields make_fields) noexcept {
        try {
            PendingEvent pending{Clock::now(), event, make_fields()};
            std::lock_guard<std::mutex> lock(mutex_);
            enqueue_locked(std::move(pending));
        } catch (...) {
            // 诊断记录失败时，保留原有 ROS 回调行为。
        }
    }

    void record_status(const BackendStatus &status) noexcept {
        try {
            const auto timestamp = Clock::now();
            auto state = status_fields(status);
            std::lock_guard<std::mutex> lock(mutex_);
            if (state == last_status_) {
                return;
            }
            auto fields = state;
            fields["stamp"] = {{"sec", status.stamp.sec}, {"nanosec", status.stamp.nanosec}};
            if (enqueue_locked({timestamp, "backend_status", std::move(fields)})) {
                last_status_ = std::move(state);
            }
        } catch (...) {
            // 即使记录失败，状态仍由节点正常发布。
        }
    }

private:
    bool enqueue_locked(PendingEvent pending) {
        if (stopping_) {
            return false;
        }
        if (pending_.size() >= kMaxPendingEvents) {
            ++dropped_;
            return false;
        }
        pending_.push_back(std::move(pending));
        ready_.notify_one();
        return true;
    }

    void write_event(PendingEvent &event) noexcept {
        try {
            event.fields["timestamp"] = format_timestamp(event.timestamp);
            event.fields["node"] = node_;
            event.fields["event"] = event.name;
            // ROS 的 JSON 字段仍是字符串，方便同时追查有效和无效输入。
            file_ << event.fields.dump(-1, ' ', false, Json::error_handler_t::replace) << '\n';
        } catch (...) {
            // 后台写入或编码失败不会中断节点。
        }
    }

    void write_loop() noexcept {
        try {
            std::vector<PendingEvent> batch;
            for (;;) {
                std::size_t dropped = 0;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    ready_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
                    if (stopping_ && pending_.empty() && dropped_ == 0) {
                        return;
                    }
                    pending_.swap(batch);
                    dropped = std::exchange(dropped_, 0);
                }
                // 交换队列后释放互斥锁，文件写入和刷新都在后台线程完成。
                for (auto &event : batch) {
                    write_event(event);
                }
                batch.clear();
                if (dropped > 0) {
                    PendingEvent report{
                        Clock::now(), "recorder_dropped", {{"dropped_count", dropped}}};
                    write_event(report);
                }
                file_.flush();
            }
        } catch (...) {
            // 记录线程退出时，节点仍可继续运行。
        }
    }

    const std::filesystem::path path_;
    const std::string node_;
    std::ofstream file_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::vector<PendingEvent> pending_;
    Json last_status_;
    std::size_t dropped_{0};
    bool stopping_{false};
    std::thread worker_;
};

DebugRecorder::DebugRecorder(const std::string &directory, const std::string &node_name)
    : impl_(std::make_unique<Impl>(directory, node_name)) {}

DebugRecorder::~DebugRecorder() = default;

const std::filesystem::path &DebugRecorder::path() const noexcept {
    return impl_->path();
}

void DebugRecorder::record_goal(
    const rclcpp_action::GoalUUID &uuid,
    const ExecuteBackendJob::Goal &goal,
    bool accepted,
    const char *reason) noexcept {
    impl_->record("action_goal", [&] {
        auto fields = action_identity(uuid, goal);
        fields["backend"] = goal.backend;
        fields["dry_run"] = goal.dry_run;
        auto instructions = Json::array();
        for (const auto &instruction : goal.instructions) {
            instructions.push_back(instruction_fields(instruction));
        }
        fields["instructions"] = std::move(instructions);
        fields["accepted"] = accepted;
        fields["reason"] = reason == nullptr ? "" : reason;
        return fields;
    });
}

void DebugRecorder::record_cancel(
    const rclcpp_action::GoalUUID &uuid,
    const ExecuteBackendJob::Goal &goal,
    bool accepted) noexcept {
    impl_->record("action_cancel", [&] {
        auto fields = action_identity(uuid, goal);
        fields["accepted"] = accepted;
        return fields;
    });
}

void DebugRecorder::record_feedback(
    const rclcpp_action::GoalUUID &uuid,
    const ExecuteBackendJob::Goal &goal,
    const ExecuteBackendJob::Feedback &feedback) noexcept {
    impl_->record("action_feedback", [&] {
        auto fields = action_identity(uuid, goal);
        fields["stage"] = feedback.stage;
        fields["current_index"] = feedback.current_index;
        fields["progress"] = feedback.progress;
        fields["current_instruction_id"] = feedback.current_instruction_id;
        fields["message"] = feedback.message;
        return fields;
    });
}

void DebugRecorder::record_result(
    const rclcpp_action::GoalUUID &uuid,
    const ExecuteBackendJob::Goal &goal,
    const ExecuteBackendJob::Result &result,
    const char *action_state) noexcept {
    impl_->record("action_result", [&] {
        auto fields = action_identity(uuid, goal);
        fields["action_state"] = action_state == nullptr ? "" : action_state;
        fields["success"] = result.success;
        fields["error_code"] = result.error_code;
        fields["message"] = result.message;
        fields["backend_job_id"] = result.backend_job_id;
        fields["artifact_manifest_json"] = result.artifact_manifest_json;
        fields["result_json"] = result.result_json;
        return fields;
    });
}

void DebugRecorder::record_stop_request(const StopBackendJob::Request &request) noexcept {
    impl_->record("stop_request", [&] {
        return Json{{"job_id", request.job_id}};
    });
}

void DebugRecorder::record_stop_response(
    const StopBackendJob::Request &request,
    const StopBackendJob::Response &response) noexcept {
    impl_->record("stop_response", [&] {
        return Json{
            {"job_id", request.job_id},
            {"accepted", response.accepted},
            {"message", response.message},
        };
    });
}

void DebugRecorder::record_status(const BackendStatus &status) noexcept {
    impl_->record_status(status);
}

}  // namespace utils
