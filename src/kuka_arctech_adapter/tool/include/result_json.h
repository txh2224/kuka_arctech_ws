#ifndef TOOL_RESULT_JSON_H
#define TOOL_RESULT_JSON_H

#include <string>

#include <atomic_adapter_interfaces/action/execute_backend_job.hpp>
#include <nlohmann/json_fwd.hpp>

#include "error_code.h"
#include "job_ir.h"

namespace tool {

// 初始化作业结果，写入契约版本、作业 ID、追踪 ID 和后端模式。
// 执行器按 Goal 的原始顺序向 step_results 追加逐步结果。
nlohmann::json make_job_result_json(
    const std::string &job_id,
    const std::string &trace_id,
    const std::string &mode);

// 作业在执行前失败时，根据原始 Goal 生成 result_json 字符串。
// 每条指令标记为 SKIPPED，并携带作业错误码和原因。
std::string make_rejected_result_json(
    const atomic_adapter_interfaces::action::ExecuteBackendJob::Goal &goal,
    const std::string &mode,
    ErrorCode code,
    const std::string &message);

// 生成失败、取消或跳过的步骤结果。payload 为空对象，evidence 记录状态时间。
// raw_evidence 非空时，将底层故障或导出路径等原始信息写入 evidence.raw。
nlohmann::json make_terminal_step_json(
    const JobStep &step,
    const std::string &mode,
    const std::string &state,
    ErrorCode code,
    const std::string &message,
    const std::string &raw_evidence);

// 生成 Mock 步骤的成功结果。payload 按指令类型填充模拟数据，evidence 标记模拟来源。
nlohmann::json make_mock_step_json(
    const JobStep &step,
    const std::string &mode,
    const std::string &message);

}  // namespace tool

#endif  // TOOL_RESULT_JSON_H
