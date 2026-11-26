#include "result_json.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <type_traits>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>

#include "pose_conversion.h"

namespace utils {
namespace {

// 生成带毫秒的 UTC 时间戳，供 observed_at 字段使用。
std::string utc_now_iso8601() {
    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()) %
                    1000;
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
    gmtime_r(&t, &utc_tm);

    std::ostringstream oss;
    oss << std::put_time(&utc_tm, "%Y-%m-%dT%H:%M:%S")
        << '.' << std::setw(3) << std::setfill('0') << ms.count() << 'Z';
    return oss.str();
}

// A/B 接口中的位置用米表示，四元数分量按 xyzw 排列。
nlohmann::json kuka_pose_to_ros_json(const KukaPose &pose) {
    const auto ros_pose = kuka_to_ros_pose(pose);
    return nlohmann::json{
        {"position_m", ros_pose.position_m},
        {"quaternion_xyzw", ros_pose.quaternion_xyzw},
    };
}

nlohmann::json make_evidence_json(const std::string &mode) {
    return nlohmann::json{{"mode", mode}, {"simulated", mode == "MOCK"}};
}

// 统一逐步结果的公共字段。A 组通过 index 和 instance_id 关联原始指令实例。
nlohmann::json make_step_result_json(
    uint32_t index,
    const std::string &instance_id,
    const std::string &instruction_id,
    const std::string &state,
    ErrorCode code,
    const std::string &message,
    nlohmann::json payload,
    nlohmann::json evidence) {
    return nlohmann::json{
        {"index", index},
        {"instance_id", instance_id},
        {"instruction_id", instruction_id},
        {"state", state},
        {"payload", std::move(payload)},
        {"error_code", to_error_code_string(code)},
        {"message", message},
        {"evidence", std::move(evidence)},
    };
}

// 按 IR 步骤类型生成模拟业务数据和完成条件，两者共用同一观测时间。
void build_mock_step_output(
    const JobStep &job_step,
    const std::string &mode,
    const std::string &observed_at,
    nlohmann::json &out_payload,
    nlohmann::json &out_evidence) {
    out_evidence = make_evidence_json(mode);
    out_evidence["observed_at"] = observed_at;

    std::visit(
        [&](const auto &step) {
            using T = std::decay_t<decltype(step)>;
            if constexpr (std::is_same_v<T, MoveJointStep>) {
                out_payload = nlohmann::json{
                    {"joint_names", step.joint_names},
                    {"actual_rad", step.target_rad},
                    {"max_error_rad", 0.0},
                    {"settled", true},
                    {"observed_at", observed_at},
                };
                out_evidence["completion_condition"] = "target_reached";
                out_evidence["settle_duration_ms"] = job_step.settle_duration_ms;
                out_evidence["settle_velocity_rad_s"] = 0.0;
            } else if constexpr (std::is_same_v<T, MoveLinearStep>) {
                out_payload = nlohmann::json{
                    {"actual_pose", kuka_pose_to_ros_json(step.target_pose)},
                    {"position_error_m", 0.0},
                    {"orientation_error_rad", 0.0},
                    {"max_path_deviation_m", 0.0},
                    {"settled", true},
                    {"observed_at", observed_at},
                };
                out_evidence["completion_condition"] = "target_reached";
                out_evidence["settle_duration_ms"] = job_step.settle_duration_ms;
            } else if constexpr (std::is_same_v<T, FollowPathStep>) {
                const KukaPose final_pose =
                    step.waypoints.empty() ? KukaPose{} : step.waypoints.back();
                out_payload = nlohmann::json{
                    {"completed_waypoints", step.waypoints.size()},
                    {"actual_pose", kuka_pose_to_ros_json(final_pose)},
                    {"max_path_deviation_m", 0.0},
                    {"observed_at", observed_at},
                };
                out_evidence["completion_condition"] = "path_completed";
            } else if constexpr (std::is_same_v<T, RecipeSelectStep>) {
                out_payload = nlohmann::json{
                    {"active_program_id", step.program_id},
                    {"observed_at", observed_at},
                };
                out_evidence["completion_condition"] = "program_active";
            } else if constexpr (std::is_same_v<T, ArcStartStep>) {
                out_payload = nlohmann::json{
                    {"arc_established", true},
                    {"observed_at", observed_at},
                };
                out_evidence["completion_condition"] = "arc_established";
            } else if constexpr (std::is_same_v<T, ArcStopStep>) {
                out_payload = nlohmann::json{
                    {"arc_stopped", true},
                    {"observed_at", observed_at},
                };
                out_evidence["completion_condition"] = "arc_stopped";
            } else if constexpr (std::is_same_v<T, MeasureProcessStep>) {
                nlohmann::json readings = nlohmann::json::array();
                for (const auto &ch : step.channels) {
                    if (ch == "current") {
                        readings.push_back({
                            {"channel_id", "current"},
                            {"value", 182.4},
                            {"unit", "A"},
                            {"valid", true},
                        });
                    } else if (ch == "voltage") {
                        readings.push_back({
                            {"channel_id", "voltage"},
                            {"value", 22.7},
                            {"unit", "V"},
                            {"valid", true},
                        });
                    } else {
                        readings.push_back({
                            {"channel_id", ch},
                            {"value", 0.0},
                            {"unit", ""},
                            {"valid", true},
                        });
                    }
                }
                out_payload = nlohmann::json{
                    {"readings", std::move(readings)},
                    {"observed_at", observed_at},
                };
                out_evidence["completion_condition"] = "measurements_returned";
            }
        },
        job_step.payload);
}

}  // namespace

nlohmann::json make_job_result_json(
    const std::string &job_id,
    const std::string &trace_id,
    const std::string &mode) {
    return nlohmann::json{
        {"contract_version", "1.0"},
        {"job_id", job_id},
        {"trace_id", trace_id},
        {"mode", mode},
        {"step_results", nlohmann::json::array()},
    };
}

std::string make_rejected_result_json(
    const atomic_adapter_interfaces::action::ExecuteBackendJob::Goal &goal,
    const std::string &mode,
    ErrorCode code,
    const std::string &message) {
    auto result = make_job_result_json(goal.job_id, goal.trace_id, mode);
    auto &steps = result["step_results"];
    // 同一次作业拒绝产生的所有跳过记录共用拒绝时刻。
    const std::string observed_at = utc_now_iso8601();
    for (std::size_t i = 0; i < goal.instructions.size(); ++i) {
        const auto &instruction = goal.instructions[i];
        auto evidence = make_evidence_json(mode);
        evidence["observed_at"] = observed_at;
        steps.push_back(make_step_result_json(
            static_cast<uint32_t>(i),
            instruction.instance_id,
            instruction.instruction_id,
            "SKIPPED",
            code,
            message,
            nlohmann::json::object(),
            std::move(evidence)));
    }
    return result.dump();
}

nlohmann::json make_terminal_step_json(
    const JobStep &step,
    const std::string &mode,
    const std::string &state,
    ErrorCode code,
    const std::string &message,
    const std::string &raw_evidence) {
    auto evidence = make_evidence_json(mode);
    evidence["observed_at"] = utc_now_iso8601();
    if (!raw_evidence.empty()) {
        evidence["raw"] = raw_evidence;
    }
    return make_step_result_json(
        step.index,
        step.instance_id,
        step.instruction_id,
        state,
        code,
        message,
        nlohmann::json::object(),
        std::move(evidence));
}

nlohmann::json make_mock_step_json(
    const JobStep &step,
    const std::string &mode,
    const std::string &message) {
    const std::string observed_at = utc_now_iso8601();
    nlohmann::json payload;
    nlohmann::json evidence;
    build_mock_step_output(step, mode, observed_at, payload, evidence);
    return make_step_result_json(
        step.index,
        step.instance_id,
        step.instruction_id,
        "SUCCEEDED",
        ErrorCode::OK,
        message,
        std::move(payload),
        std::move(evidence));
}

}  // namespace utils
