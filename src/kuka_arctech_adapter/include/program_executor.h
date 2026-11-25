#ifndef PROGRAM_EXECUTOR_H
#define PROGRAM_EXECUTOR_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "error_code.h"
#include "job_ir.h"
#include "krc_transport.h"

// Feedback.stage 约定（对齐 A/B 通信定义与规格书第 11 节）
enum class ExecutionStage : uint8_t {
    IDLE = 0,
    VALIDATING = 1,
    COMPILING = 2,
    RUNNING = 3,
    WELDING = 4,
};

enum class ExecutionOutcome {
    SUCCEEDED,
    CANCELED,
    FAILED,
};

struct ExecutionResult {
    ExecutionOutcome outcome{ExecutionOutcome::FAILED};
    ErrorCode error_code{ErrorCode::OK};
    std::string message;
    std::string backend_job_id;
    std::string artifact_manifest_json{"{}"};
    std::string result_json{"{}"};
};

using FeedbackCallback = std::function<void(
    uint8_t stage,
    uint32_t current_index,
    float progress,
    const std::string &current_instruction_id,
    const std::string &message)>;

// 负责通过 IKrcTransport 执行校验通过的 IR，并按 A/B 通信定义回传进度与 step_results
class ProgramExecutor {
public:
    explicit ProgramExecutor(
        std::shared_ptr<IKrcTransport> transport,
        uint32_t step_delay_ms = 50);

    ExecutionResult execute(
        const KukaArcTechJobIR &job_ir,
        const std::atomic<bool> &stop_requested,
        const FeedbackCallback &feedback_cb,
        const ProgramPackage *package_override = nullptr);

private:
    std::shared_ptr<IKrcTransport> transport_;
    uint32_t step_delay_ms_{50};
};

#endif  // PROGRAM_EXECUTOR_H
