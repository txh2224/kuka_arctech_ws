#include "program_executor.h"
#include "result_json.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>

namespace {

ExecutionStage stage_for_step(const StepPayload &payload) {
    if (std::holds_alternative<ArcStartStep>(payload) ||
        std::holds_alternative<ArcStopStep>(payload) ||
        std::holds_alternative<MeasureProcessStep>(payload)) {
        return ExecutionStage::WELDING;
    }
    return ExecutionStage::RUNNING;
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

    auto result_doc = utils::make_job_result_json(job_ir.job_id, job_ir.trace_id, mode);

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
        result_doc["step_results"].push_back(
            utils::make_terminal_step_json(step, mode, state, code, msg, raw_evidence));
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
                    static_cast<uint8_t>(ExecutionStage::WELDING),
                    step.index,
                    progress,
                    step.instruction_id,
                    mode + " querying process measurements");
            }

            result_doc["step_results"].push_back(utils::make_mock_step_json(step, mode, "OK"));
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
            auto step_json = utils::make_terminal_step_json(
                step,
                mode,
                "SKIPPED",
                ErrorCode::KA_PROGRAM_START_FAILED,
                wait_msg,
                deploy_res.raw_evidence);
            step_json["evidence"]["adapter_state"] = "WAITING_OPERATOR";
            result_doc["step_results"].push_back(std::move(step_json));
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

        // 每次唤醒先检查停止和超时，再判断模拟步骤是否完成。
        using Clock = std::chrono::steady_clock;
        constexpr auto check_interval = std::chrono::milliseconds(10);
        const auto step_start = Clock::now();
        const auto step_finish = step_start + std::chrono::milliseconds(step_delay_ms_);
        const auto step_deadline = step.timeout_ms == 0
            ? Clock::time_point::max()
            : step_start + std::chrono::milliseconds(step.timeout_ms);

        while (true) {
            if (stop_requested.load()) {
                return handle_stop_at_step(i, "作业在执行过程中已按请求停止。");
            }
            const auto now = Clock::now();
            if (now >= step_deadline) {
                const std::string timeout_evidence = "timeout_ms=" + std::to_string(step.timeout_ms);
                const std::string timeout_msg =
                    "步骤 " + step.instruction_id + " 执行超过 " + timeout_evidence + "ms，已触发停机。";
                const auto stop_res = stop_transport_once();
                if (!stop_res.ok) {
                    const std::string combined_msg =
                        timeout_msg + "；且后续停止失败：" + stop_res.message;
                    const std::string combined_ev =
                        timeout_evidence + "; stop_failed=" + stop_res.raw_evidence;
                    append_step_terminal(
                        step, "FAILED", ErrorCode::KA_STOP_FAILED, combined_msg, combined_ev);
                    fill_remaining_steps(
                        i + 1, "SKIPPED", ErrorCode::OK, "因前序步骤超时且停机失败而跳过。");
                    return finalize_failed(ErrorCode::KA_STOP_FAILED, combined_msg, combined_ev);
                }
                append_step_terminal(
                    step, "FAILED", ErrorCode::KA_PROGRAM_TIMEOUT, timeout_msg, timeout_evidence);
                fill_remaining_steps(
                    i + 1, "SKIPPED", ErrorCode::OK, "因前序步骤执行超时而跳过。");
                return finalize_failed(ErrorCode::KA_PROGRAM_TIMEOUT, timeout_msg, timeout_evidence);
            }
            if (now >= step_finish) {
                break;
            }
            std::this_thread::sleep_until(std::min({now + check_interval, step_deadline, step_finish}));
        }

        result_doc["step_results"].push_back(utils::make_mock_step_json(
            step, mode, mode == "MOCK" ? "simulated completion" : "completed"));
    }

    transport_->set_runtime_state("IDLE", "ARC_OFF", mode + " completed");

    exec_res.outcome = ExecutionOutcome::SUCCEEDED;
    exec_res.error_code = ErrorCode::OK;
    exec_res.message = mode + " completed";
    exec_res.result_json = result_doc.dump();
    return exec_res;
}
