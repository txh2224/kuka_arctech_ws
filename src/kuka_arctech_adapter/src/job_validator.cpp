#include "job_validator.h"

bool JobValidator::validate(
    const atomic_adapter_interfaces::action::ExecuteBackendJob::Goal &goal,
    std::string &error_message) const {
    error_message.clear();

    if (goal.job_id.empty()) {
        error_message = "job_id 不能为空。";
        return false;
    }
    if (goal.trace_id.empty()) {
        error_message = "trace_id 不能为空。";
        return false;
    }
    if (goal.backend != "kuka_arctech") {
        error_message = "不支持的后端类型：" + goal.backend;
        return false;
    }
    if (goal.instructions.empty()) {
        error_message = "原子指令序列不能为空。";
        return false;
    }

    return true;
}
