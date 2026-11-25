#include "program_executor.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>
#include <variant>

#include <nlohmann/json.hpp>

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

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

nlohmann::json kuka_pose_to_ros_json(const KukaPose &pose) {
    const double yaw = pose.a * kDegToRad * 0.5;
    const double pitch = pose.b * kDegToRad * 0.5;
    const double roll = pose.c * kDegToRad * 0.5;

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);

    const double qw = cr * cp * cy + sr * sp * sy;
    const double qx = sr * cp * cy - cr * sp * sy;
    const double qy = cr * sp * cy + sr * cp * sy;
    const double qz = cr * cp * sy - sr * sp * cy;

    return nlohmann::json{
        {"position_m", {pose.x / 1000.0, pose.y / 1000.0, pose.z / 1000.0}},
        {"quaternion_xyzw", {qx, qy, qz, qw}},
    };
}

ExecutionStage stage_for_step(const StepPayload &payload) {
    if (std::holds_alternative<ArcStartStep>(payload) ||
        std::holds_alternative<ArcStopStep>(payload) ||
        std::holds_alternative<MeasureProcessStep>(payload)) {
        return ExecutionStage::WELDING;
    }
    return ExecutionStage::RUNNING;
}

void build_mock_step_output(
    const JobStep &job_step,
    const std::string &mode,
    const std::string &observed_at,
    nlohmann::json &out_payload,
    nlohmann::json &out_evidence) {
    const bool simulated = (mode == "MOCK");
    out_evidence = nlohmann::json{
        {"mode", mode},
        {"simulated", simulated},
        {"observed_at", observed_at},
    };

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

ProgramExecutor::ProgramExecutor(
    std::shared_ptr<IKrcTransport> transport,
    uint32_t step_delay_ms)
    : transport_(std::move(transport)), step_delay_ms_(step_delay_ms) {}

ExecutionResult ProgramExecutor::execute(
    const KukaArcTechJobIR &job_ir,
    const std::atomic<bool> &stop_requested,
    const FeedbackCallback &feedback_cb,
    const ProgramPackage *package_override) {
    ExecutionResult exec_res;
    exec_res.backend_job_id = "b_" + job_ir.job_id;
    const std::string mode = transport_ ? transport_->mode_name() : "MOCK";

    nlohmann::json result_doc{
        {"contract_version", "1.0"},
        {"job_id", job_ir.job_id},
        {"trace_id", job_ir.trace_id},
        {"mode", mode},
        {"step_results", nlohmann::json::array()},
    };

    // 由执行线程独占底层启停，先切到可观测的 STOPPING 状态再停机
    auto stop_transport_once = [&]() -> CommandResult {
        transport_->set_runtime_state("STOPPING", "ARC_OFF", "Stopping active program");
        auto res = transport_->stop_program();
        if (!res.ok) {
            // 任何停机未确认原因（包括停机期间断连 KA_KRC_UNREACHABLE）均归并为 KA_STOP_FAILED 以便上层锁槽
            const std::string orig_code = to_error_code_string(res.error_code);
            if (!orig_code.empty() && res.error_code != ErrorCode::KA_STOP_FAILED) {
                res.raw_evidence =
                    "stop_cause=" + orig_code +
                    (res.raw_evidence.empty() ? "" : ("; " + res.raw_evidence));
            }
            res.error_code = ErrorCode::KA_STOP_FAILED;
        }
        return res;
    };

    auto append_step_terminal = [&](const JobStep &step,
                                    const std::string &state,
                                    ErrorCode code,
                                    const std::string &msg,
                                    const std::string &raw_evidence = "") {
        nlohmann::json ev{
            {"mode", mode},
            {"simulated", mode == "MOCK"},
            {"observed_at", utc_now_iso8601()},
        };
        if (!raw_evidence.empty()) {
            ev["raw"] = raw_evidence;
        }
        result_doc["step_results"].push_back({
            {"index", step.index},
            {"instance_id", step.instance_id},
            {"instruction_id", step.instruction_id},
            {"state", state},
            {"payload", nlohmann::json::object()},
            {"error_code", to_error_code_string(code)},
            {"message", msg},
            {"evidence", std::move(ev)},
        });
    };

    auto fill_remaining_steps = [&](std::size_t start_index,
                                    const std::string &state,
                                    ErrorCode code,
                                    const std::string &msg) {
        for (std::size_t k = start_index; k < job_ir.steps.size(); ++k) {
            append_step_terminal(job_ir.steps[k], state, code, msg);
        }
    };

    auto finalize_failed = [&](ErrorCode code, const std::string &msg,
                               const std::string &raw_evidence = "") {
        if (!raw_evidence.empty()) {
            result_doc["evidence"] = {{"raw", raw_evidence}};
        }
        exec_res.outcome = ExecutionOutcome::FAILED;
        exec_res.error_code = code;
        exec_res.message = msg;
        exec_res.result_json = result_doc.dump();
        return exec_res;
    };

    auto finalize_canceled = [&](const std::string &msg) {
        exec_res.outcome = ExecutionOutcome::CANCELED;
        exec_res.error_code = ErrorCode::OK;
        exec_res.message = msg;
        exec_res.result_json = result_doc.dump();
        return exec_res;
    };

    auto handle_stop_at_step = [&](std::size_t step_idx, const std::string &cancel_msg) {
        const auto &step = job_ir.steps[step_idx];
        const auto stop_res = stop_transport_once();
        if (!stop_res.ok) {
            append_step_terminal(
                step, "FAILED", stop_res.error_code, stop_res.message, stop_res.raw_evidence);
            fill_remaining_steps(
                step_idx + 1, "SKIPPED", ErrorCode::OK, "因前序步骤停机失败而跳过。");
            return finalize_failed(stop_res.error_code, stop_res.message, stop_res.raw_evidence);
        }
        append_step_terminal(step, "CANCELED", ErrorCode::OK, cancel_msg);
        fill_remaining_steps(step_idx + 1, "SKIPPED", ErrorCode::OK, "因作业已停止而未执行。");
        return finalize_canceled(cancel_msg);
    };

    if (!transport_) {
        fill_remaining_steps(0, "SKIPPED", ErrorCode::KA_KRC_UNREACHABLE, "KRC Transport 未初始化。");
        return finalize_failed(ErrorCode::KA_KRC_UNREACHABLE, "KRC Transport 未初始化。");
    }

    // 封住未接入真实采集的 Transport（如未完成的 VERIFIED）误走模拟输出的安全门
    if (mode != "MOCK" && mode != "MANUAL") {
        const std::string msg =
            "当前 Transport 模式 (" + mode + ") 尚未接入真实执行反馈采集，禁止使用模拟结果替代。";
        fill_remaining_steps(0, "SKIPPED", ErrorCode::KA_PROGRAM_START_FAILED, msg);
        return finalize_failed(ErrorCode::KA_PROGRAM_START_FAILED, msg);
    }

    // 启动前先检查是否已收到停止/取消请求，避免取消后仍启动程序
    if (stop_requested.load()) {
        const std::string msg = "作业在启动前已按请求取消。";
        fill_remaining_steps(0, "CANCELED", ErrorCode::OK, msg);
        return finalize_canceled(msg);
    }

    // 1. 只读查询路径（全部为 WLD-024 测量步骤）：不部署、不启动 KRL 运动程序
    if (is_read_only_query_job(job_ir)) {
        result_doc["query_only"] = true;
        exec_res.artifact_manifest_json = "{}";

        if (mode == "MANUAL") {
            const std::string msg =
                "Manual 模式为离线程序包导出，未连接真实 KRC 控制器，无法执行只读在线测量（WLD-024）。";
            fill_remaining_steps(0, "SKIPPED", ErrorCode::KA_KRC_UNREACHABLE, msg);
            return finalize_failed(ErrorCode::KA_KRC_UNREACHABLE, msg);
        }

        if (const auto probe_res = transport_->probe(); !probe_res.connected) {
            fill_remaining_steps(0, "SKIPPED", probe_res.error_code, probe_res.message);
            return finalize_failed(probe_res.error_code, probe_res.message);
        }

        const std::size_t total_steps = job_ir.steps.size();
        for (std::size_t i = 0; i < total_steps; ++i) {
            const auto &step = job_ir.steps[i];
            if (stop_requested.load()) {
                append_step_terminal(step, "CANCELED", ErrorCode::OK, "只读查询在执行前已取消。");
                fill_remaining_steps(i + 1, "SKIPPED", ErrorCode::OK, "因作业已取消而未执行。");
                return finalize_canceled("只读查询作业已取消。");
            }

            if (feedback_cb) {
                const float progress =
                    static_cast<float>(i + 1) / static_cast<float>(total_steps);
                feedback_cb(
                    static_cast<uint8_t>(ExecutionStage::RUNNING),
                    step.index,
                    progress,
                    step.instruction_id,
                    mode + " querying process measurements");
            }

            const std::string observed_at = utc_now_iso8601();
            nlohmann::json payload;
            nlohmann::json evidence;
            build_mock_step_output(step, mode, observed_at, payload, evidence);
            result_doc["step_results"].push_back({
                {"index", step.index},
                {"instance_id", step.instance_id},
                {"instruction_id", step.instruction_id},
                {"state", "SUCCEEDED"},
                {"payload", payload},
                {"error_code", ""},
                {"message", "OK"},
                {"evidence", evidence},
            });
        }

        exec_res.outcome = ExecutionOutcome::SUCCEEDED;
        exec_res.error_code = ErrorCode::OK;
        exec_res.message = mode + " 只读测量查询完成。";
        exec_res.result_json = result_doc.dump();
        return exec_res;
    }

    // 2. 探活（MANUAL 模式检查本地导出目录就绪，且 controller_connected 始终为 false）
    const auto probe_res = transport_->probe();
    if (mode == "MANUAL") {
        if (probe_res.error_code != ErrorCode::OK) {
            fill_remaining_steps(0, "SKIPPED", probe_res.error_code, probe_res.message);
            return finalize_failed(probe_res.error_code, probe_res.message);
        }
    } else if (!probe_res.connected) {
        fill_remaining_steps(0, "SKIPPED", probe_res.error_code, probe_res.message);
        return finalize_failed(probe_res.error_code, probe_res.message);
    }

    // 3. 准备程序包并部署（路线 B 接入模板引擎后可通过 package_override 传入真实 SRC/DAT）
    ProgramPackage pkg;
    if (package_override != nullptr) {
        pkg = *package_override;
    } else {
        pkg.program_name = job_ir.program_name;
        pkg.manifest_json = "{}";
    }
    exec_res.artifact_manifest_json = pkg.manifest_json;

    if (feedback_cb) {
        feedback_cb(
            static_cast<uint8_t>(ExecutionStage::COMPILING),
            0,
            0.2f,
            job_ir.steps.empty() ? "" : job_ir.steps.front().instruction_id,
            mode + " deploying package");
    }

    if (stop_requested.load()) {
        const std::string msg = "作业在导出/部署前已按请求停止。";
        fill_remaining_steps(0, "CANCELED", ErrorCode::OK, msg);
        return finalize_canceled(msg);
    }

    const auto deploy_res = transport_->deploy(pkg);
    if (!deploy_res.ok) {
        fill_remaining_steps(0, "SKIPPED", deploy_res.error_code, deploy_res.message);
        return finalize_failed(deploy_res.error_code, deploy_res.message, deploy_res.raw_evidence);
    }

    // MANUAL 模式：程序包已落盘导出，但未在 KRC 上执行，不能返回 SUCCEEDED / success=true，
    // 以 ABORTED + KA_PROGRAM_START_FAILED 返回导出证据并提示人工导入执行。
    if (mode == "MANUAL") {
        const std::string wait_msg =
            deploy_res.message + "（Manual 模式无法自动在 KRC 上启动，作业尚未执行完成）";
        for (const auto &step : job_ir.steps) {
            append_step_terminal(
                step,
                "WAITING_OPERATOR",
                ErrorCode::KA_PROGRAM_START_FAILED,
                wait_msg,
                deploy_res.raw_evidence);
        }
        result_doc["export_dir"] = deploy_res.raw_evidence;
        result_doc["execution_completed"] = false;
        result_doc["manual_handoff_required"] = true;
        if (feedback_cb) {
            feedback_cb(
                static_cast<uint8_t>(ExecutionStage::COMPILING),
                job_ir.steps.empty() ? 0 : job_ir.steps.back().index,
                0.9f,
                job_ir.steps.empty() ? "" : job_ir.steps.back().instruction_id,
                wait_msg);
        }
        exec_res.outcome = ExecutionOutcome::FAILED;
        exec_res.error_code = ErrorCode::KA_PROGRAM_START_FAILED;
        exec_res.message = wait_msg;
        exec_res.result_json = result_doc.dump();
        return exec_res;
    }

    if (stop_requested.load()) {
        const std::string msg = "作业在程序启动前已按请求停止。";
        fill_remaining_steps(0, "CANCELED", ErrorCode::OK, msg);
        return finalize_canceled(msg);
    }

    if (const auto select_res = transport_->select_program(pkg.program_name); !select_res.ok) {
        fill_remaining_steps(0, "SKIPPED", select_res.error_code, select_res.message);
        return finalize_failed(select_res.error_code, select_res.message, select_res.raw_evidence);
    }

    if (stop_requested.load()) {
        const std::string msg = "作业在程序启动前已按请求停止。";
        fill_remaining_steps(0, "CANCELED", ErrorCode::OK, msg);
        return finalize_canceled(msg);
    }

    if (const auto start_res = transport_->start_program(pkg.program_name); !start_res.ok) {
        fill_remaining_steps(0, "SKIPPED", start_res.error_code, start_res.message);
        return finalize_failed(start_res.error_code, start_res.message, start_res.raw_evidence);
    }

    // 3. 按序执行每一步，并按 A/B 通信定义发布 Feedback 与收集 step_results
    const std::size_t total_steps = job_ir.steps.size();
    bool arc_active = false;

    for (std::size_t i = 0; i < total_steps; ++i) {
        const auto &step = job_ir.steps[i];

        // 检查外部取消/停止请求
        if (stop_requested.load()) {
            return handle_stop_at_step(i, "作业已按请求停止。");
        }

        // 检查底层故障
        if (const auto faults = transport_->read_faults(); !faults.empty()) {
            const auto stop_res = stop_transport_once();
            transport_->set_runtime_state("FAULT", "FAULT", faults.front().message);
            const std::string orig_msg = "检测到 KRC/ArcTech 故障：" + faults.front().message;
            if (!stop_res.ok) {
                const std::string combined_msg =
                    orig_msg + "；且后续停止失败：" + stop_res.message;
                const std::string combined_ev =
                    faults.front().code + "; stop_failed=" + stop_res.raw_evidence;
                append_step_terminal(
                    step, "FAILED", ErrorCode::KA_STOP_FAILED, combined_msg, combined_ev);
                fill_remaining_steps(
                    i + 1, "SKIPPED", ErrorCode::OK, "因前序步骤故障且停机失败而跳过。");
                return finalize_failed(ErrorCode::KA_STOP_FAILED, combined_msg, combined_ev);
            }
            append_step_terminal(
                step, "FAILED", ErrorCode::KA_ARCTECH_FAULT, orig_msg, faults.front().code);
            fill_remaining_steps(i + 1, "SKIPPED", ErrorCode::OK, "因前序步骤发生故障而跳过。");
            return finalize_failed(ErrorCode::KA_ARCTECH_FAULT, orig_msg, faults.front().code);
        }

        // 遇到起弧步骤时模拟/执行焊缝许可握手，并校验返回焊缝号与请求一致
        if (std::holds_alternative<ArcStartStep>(step.payload) && job_ir.seam_id > 0) {
            const auto seam_res = transport_->set_seam_permission(job_ir.seam_id, true);
            const bool seam_ok =
                seam_res.granted && (seam_res.granted_seam_id == job_ir.seam_id);
            if (!seam_ok) {
                const ErrorCode seam_err =
                    (seam_res.error_code != ErrorCode::OK)
                        ? seam_res.error_code
                        : ErrorCode::KA_SEAM_PERMISSION_TIMEOUT;
                const std::string seam_msg =
                    !seam_res.granted
                        ? seam_res.message
                        : ("焊缝许可返回编号不匹配：请求=" + std::to_string(job_ir.seam_id) +
                           "，授予=" + std::to_string(seam_res.granted_seam_id) + "。");
                const auto stop_res = stop_transport_once();
                if (!stop_res.ok) {
                    const std::string combined_msg =
                        seam_msg + "；且后续停止失败：" + stop_res.message;
                    const std::string combined_ev =
                        "seam_permission_denied; stop_failed=" + stop_res.raw_evidence;
                    append_step_terminal(
                        step, "FAILED", ErrorCode::KA_STOP_FAILED, combined_msg, combined_ev);
                    fill_remaining_steps(
                        i + 1, "SKIPPED", ErrorCode::OK, "因前序步骤许可失败且停机失败而跳过。");
                    return finalize_failed(ErrorCode::KA_STOP_FAILED, combined_msg, combined_ev);
                }
                append_step_terminal(step, "FAILED", seam_err, seam_msg);
                fill_remaining_steps(
                    i + 1, "SKIPPED", ErrorCode::OK, "因前序步骤焊缝许可失败而跳过。");
                return finalize_failed(seam_err, seam_msg);
            }
        }

        const ExecutionStage stage = stage_for_step(step.payload);
        if (std::holds_alternative<ArcStartStep>(step.payload)) {
            arc_active = true;
        } else if (std::holds_alternative<ArcStopStep>(step.payload)) {
            arc_active = false;
        }

        transport_->set_runtime_state(
            "RUNNING",
            arc_active ? "WELDING" : "ARC_OFF",
            mode + " step active: " + step.instruction_id);

        if (feedback_cb) {
            // 部署阶段占 0.0~0.2，步骤执行阶段单调推进 0.2~1.0，避免进度回退
            const float progress =
                0.2f + 0.8f * ((static_cast<float>(i) + 0.5f) / static_cast<float>(total_steps));
            feedback_cb(
                static_cast<uint8_t>(stage),
                step.index,
                progress,
                step.instruction_id,
                mode + " step active");
        }

        // 分段小睡以便及时响应取消/停止请求与单步超时（timeout_ms）
        const auto step_start_tp = std::chrono::steady_clock::now();
        if (step_delay_ms_ > 0) {
            const uint32_t slice_ms = 10;
            uint32_t elapsed = 0;
            while (elapsed < step_delay_ms_) {
                if (stop_requested.load()) {
                    return handle_stop_at_step(i, "作业在执行过程中已按请求停止。");
                }
                if (step.timeout_ms > 0) {
                    const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                             std::chrono::steady_clock::now() - step_start_tp)
                                             .count();
                    if (wall_ms >= static_cast<int64_t>(step.timeout_ms)) {
                        const std::string timeout_msg =
                            "步骤 " + step.instruction_id + " 执行超过 timeout_ms=" +
                            std::to_string(step.timeout_ms) + "ms，已触发停机。";
                        const auto stop_res = stop_transport_once();
                        if (!stop_res.ok) {
                            const std::string combined_msg =
                                timeout_msg + "；且后续停止失败：" + stop_res.message;
                            const std::string combined_ev =
                                "timeout_ms=" + std::to_string(step.timeout_ms) +
                                "; stop_failed=" + stop_res.raw_evidence;
                            append_step_terminal(
                                step, "FAILED", ErrorCode::KA_STOP_FAILED, combined_msg, combined_ev);
                            fill_remaining_steps(
                                i + 1, "SKIPPED", ErrorCode::OK, "因前序步骤超时且停机失败而跳过。");
                            return finalize_failed(
                                ErrorCode::KA_STOP_FAILED, combined_msg, combined_ev);
                        }
                        append_step_terminal(
                            step,
                            "FAILED",
                            ErrorCode::KA_PROGRAM_TIMEOUT,
                            timeout_msg,
                            "timeout_ms=" + std::to_string(step.timeout_ms));
                        fill_remaining_steps(
                            i + 1, "SKIPPED", ErrorCode::OK, "因前序步骤执行超时而跳过。");
                        return finalize_failed(
                            ErrorCode::KA_PROGRAM_TIMEOUT,
                            timeout_msg,
                            "timeout_ms=" + std::to_string(step.timeout_ms));
                    }
                }
                const uint32_t wait_ms = std::min(slice_ms, step_delay_ms_ - elapsed);
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
                elapsed += wait_ms;
            }
        }

        // 记录步骤完成前再次确认是否收到停止请求，防止最后一步误报 SUCCEEDED
        if (stop_requested.load()) {
            return handle_stop_at_step(i, "作业在执行过程中已按请求停止。");
        }

        const std::string observed_at = utc_now_iso8601();
        nlohmann::json step_payload;
        nlohmann::json step_evidence;
        build_mock_step_output(step, mode, observed_at, step_payload, step_evidence);

        result_doc["step_results"].push_back({
            {"index", step.index},
            {"instance_id", step.instance_id},
            {"instruction_id", step.instruction_id},
            {"state", "SUCCEEDED"},
            {"payload", std::move(step_payload)},
            {"error_code", ""},
            {"message", mode == "MOCK" ? "simulated completion" : "completed"},
            {"evidence", std::move(step_evidence)},
        });
    }

    transport_->set_runtime_state("IDLE", "ARC_OFF", mode + " completed");

    exec_res.outcome = ExecutionOutcome::SUCCEEDED;
    exec_res.error_code = ErrorCode::OK;
    exec_res.message = mode + " completed";
    exec_res.result_json = result_doc.dump();
    return exec_res;
}
