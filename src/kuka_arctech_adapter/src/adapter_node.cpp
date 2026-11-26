#include "adapter_node.h"
#include "debug_recorder.h"
#include "error_code.h"
#include "manual_krc_transport.h"
#include "mock_krc_transport.h"
#include "result_json.h"
#include "terminal_color.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <functional>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>

namespace {

namespace terminal_color = utils::terminal_color;

//一个个人的小喜好，我比较喜欢看一些花花绿绿的输出来区别信息类型
const char *outcome_color(ExecutionOutcome outcome) {
    switch (outcome) {
        case ExecutionOutcome::SUCCEEDED:
            return terminal_color::kGreen;
        case ExecutionOutcome::CANCELED:
            return terminal_color::kYellow;
        case ExecutionOutcome::FAILED:
            return terminal_color::kRed;
    }
    return terminal_color::kReset;
}

std::shared_ptr<IKrcTransport> create_transport_from_param(rclcpp::Node &node) {
    const std::string raw_mode = node.declare_parameter<std::string>("transport_mode", "mock");
    const std::string manual_export_dir =
        node.declare_parameter<std::string>("manual_export_dir", "output");

    std::string mode = raw_mode;
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    // 目前仅支持 mock 和 manual 模式，verified 模式尚未接入现场真机接口
    if (mode == "mock") {
        return std::make_shared<MockKrcTransport>();
    }
    if (mode == "manual") {
        return std::make_shared<ManualKrcTransport>(manual_export_dir);
    }
    if (mode == "verified") {
        throw std::invalid_argument(
            "transport_mode='verified' 尚未接入现场真机接口，当前支持 'mock' 或 'manual' 模式。");
    }
    throw std::invalid_argument(
        "未知的 transport_mode='" + raw_mode + "'，允许值：mock / manual（后续支持 verified）。");
}

// 创建模板引擎，若 templates_dir 为空则使用内置模板
TemplateEngine create_template_engine_from_param(rclcpp::Node &node) {
    const std::string templates_dir = node.declare_parameter<std::string>("templates_dir", "");
    if (!templates_dir.empty()) {
        return TemplateEngine(templates_dir);
    }
    return TemplateEngine();
}

}  // namespace

AdapterNode::AdapterNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("adapter_node", options),
      template_engine_(create_template_engine_from_param(*this)),
      transport_(create_transport_from_param(*this)),
      program_executor_(transport_) {
    const auto debug_dir = declare_parameter<std::string>("debug_dir", "debug");
    if (!debug_dir.empty()) {
        try {
            debug_recorder_ =
                std::make_unique<utils::DebugRecorder>(debug_dir, get_fully_qualified_name());
            RCLCPP_INFO(get_logger(), "调试记录：%s", debug_recorder_->path().c_str());
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "无法初始化调试记录：%s", e.what());
        }
    }

    //这里创建动作Server，绑定回调函数
    execute_action_ = rclcpp_action::create_server<ExecuteBackendJob>(
        this,
        "/adapter/kuka_arctech/execute_job",
        std::bind(&AdapterNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AdapterNode::cancel_callback, this, std::placeholders::_1),
        std::bind(&AdapterNode::accepted_callback, this, std::placeholders::_1)
    );

    //创建状态广播话题与停止服务
    status_pub_ = create_publisher<BackendStatus>("/adapter/kuka_arctech/status", 10);
    stop_service_ = create_service<StopBackendJob>(
        "/adapter/kuka_arctech/stop_job",
        std::bind(
            &AdapterNode::stop_job_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&AdapterNode::publish_status, this));

    const char *mode_color =
        (transport_->mode_name() == "MANUAL") ? terminal_color::kMagenta : terminal_color::kGreen;
    RCLCPP_INFO(
        get_logger(),
        "%sAdapterNode initialized (mode=%s).%s",
        mode_color,
        transport_->mode_name().c_str(),
        terminal_color::kReset);
}

AdapterNode::~AdapterNode() {
    //释放当前任务的线程
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_job_) {
            current_job_->stop_requested.store(true);
        }
    }

    if (job_thread_.joinable()) {
        job_thread_.join();
    }
}

void AdapterNode::execute_job(
    std::shared_ptr<active_job> job,
    std::shared_ptr<ExecuteGoalHandle> goal_handle) {
    //这里是执行任务的函数，需要单独开线程来执行，避免阻塞动作的回调线程
    RCLCPP_INFO(
        get_logger(),
        "%s开始执行任务: %s%s",
        terminal_color::kCyan,
        job->job_id.c_str(),
        terminal_color::kReset);

    const auto goal = goal_handle->get_goal();

    auto result = std::make_shared<ExecuteBackendJob::Result>();
    result->success = false;
    result->backend_job_id = "b_" + goal->job_id;
    result->artifact_manifest_json = "{}";

    //创建反馈回调函数，绑定当前任务的 Action Goal
    const auto feedback_callback = make_feedback_callback(goal_handle);

    ExecutionOutcome outcome = ExecutionOutcome::FAILED;
    std::string error_message;
    
    //检查当前任务是否合法

    publish_feedback(
        *goal_handle,
        static_cast<uint8_t>(ExecutionStage::VALIDATING),
        0,
        0.0f,
        goal->instructions.empty() ? "" : goal->instructions.front().instruction_id,
        "Validating job");

    //解析
    auto err = job_validator_.validate(*goal, error_message);

    if (err != ErrorCode::OK) {
        RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
        result->error_code = to_error_code_string(err);
        result->message = error_message;
        result->result_json = make_rejected_result_json(*goal, err, error_message);
    } else {
        //这里先把任务解析为IR
        KukaArcTechJobIR job_ir;
        err = semantic_mapper_.map(*goal, job_ir, error_message);
        if (err != ErrorCode::OK) {
            RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
            result->error_code = to_error_code_string(err);
            result->message = error_message;
            result->result_json = make_rejected_result_json(*goal, err, error_message);
        } else {
            //然后再检查IR是否合法
            err = job_validator_.validate_ir(job_ir, error_message);
            if (err != ErrorCode::OK) {
                RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
                result->error_code = to_error_code_string(err);
                result->message = error_message;
                result->result_json = make_rejected_result_json(*goal, err, error_message);
            } else if (is_read_only_query_job(job_ir)) {
                // 沟槽的，你整个这玩意干嘛？
                // 只读测量作业（如单条 WLD-024）直接走查询路径，不渲染也不部署 KRL 运动程序
                const auto exec_res = program_executor_.execute(
                    job_ir,
                    job->stop_requested,
                    feedback_callback,
                    nullptr);
                outcome = exec_res.outcome;
                result->success = (exec_res.outcome == ExecutionOutcome::SUCCEEDED);
                result->error_code = to_error_code_string(exec_res.error_code);
                result->message = exec_res.message;
                result->backend_job_id = exec_res.backend_job_id;
                result->artifact_manifest_json = exec_res.artifact_manifest_json;
                result->result_json = exec_res.result_json;

                RCLCPP_INFO(
                    get_logger(),
                    "%s只读查询任务结束: %s (success=%s, error_code=%s) %s%s",
                    outcome_color(outcome),
                    goal->job_id.c_str(),
                    result->success ? "true" : "false",
                    result->error_code.empty() ? "NONE" : result->error_code.c_str(),
                    result->message.c_str(),
                    terminal_color::kReset);

            } else {
                // 通过黄金模板生成 .src/.dat 并做静态校验和 manifest 生成
                publish_feedback(
                    *goal_handle,
                    static_cast<uint8_t>(ExecutionStage::COMPILING),
                    0,
                    0.1f,
                    job_ir.steps.empty() ? "" : job_ir.steps.front().instruction_id,
                    "Rendering KRL template and running static checks");

                ProgramPackage pkg;
                err = template_engine_.render(job_ir, pkg, error_message);
                if (err != ErrorCode::OK) {
                    RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
                    result->error_code = to_error_code_string(err);
                    result->message = error_message;
                    result->result_json = make_rejected_result_json(*goal, err, error_message);
                } else {
                    const std::string mode = transport_ ? transport_->mode_name() : "MOCK";
                    err = static_checker_.check_and_build_manifest(
                        job_ir, mode, pkg, error_message);
                    if (err != ErrorCode::OK) {
                        RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
                        result->error_code = to_error_code_string(err);
                        result->message = error_message;
                        result->result_json = make_rejected_result_json(*goal, err, error_message);
                    } else {
                        //通过KRC Transport执行作业并收集逐步结果
                        const auto exec_res = program_executor_.execute(
                            job_ir,
                            job->stop_requested,
                            feedback_callback,
                            &pkg);
                        outcome = exec_res.outcome;
                        result->success = (exec_res.outcome == ExecutionOutcome::SUCCEEDED);
                        result->error_code = to_error_code_string(exec_res.error_code);
                        result->message = exec_res.message;
                        result->backend_job_id = exec_res.backend_job_id;
                        result->artifact_manifest_json = exec_res.artifact_manifest_json;
                        result->result_json = exec_res.result_json;

                        RCLCPP_INFO(
                            get_logger(),
                            "%s任务结束: %s (success=%s, error_code=%s) %s%s",
                            outcome_color(outcome),
                            goal->job_id.c_str(),
                            result->success ? "true" : "false",
                            result->error_code.empty() ? "NONE" : result->error_code.c_str(),
                            result->message.c_str(),
                            terminal_color::kReset);

                    }
                }
            }
        }
    }

    // 在锁内原子提交终态标志；若停机未确认（KA_STOP_FAILED），保留任务槽禁止接新单，直到再次停机确认。
    const bool stop_unconfirmed =
        (result->error_code == to_error_code_string(ErrorCode::KA_STOP_FAILED));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        job->terminal.store(true);
        if (stop_unconfirmed) {
            job->stop_failed.store(true);
            RCLCPP_ERROR(
                get_logger(),
                "任务 %s 停机失败（KA_STOP_FAILED），已锁定任务槽，禁止接受新任务直到停机确认。",
                job->job_id.c_str());
        } else {
            current_job_.reset();
        }
    }

    finish_action(*goal_handle, result, outcome);
}

std::string AdapterNode::make_rejected_result_json(
    const ExecuteBackendJob::Goal &goal,
    ErrorCode code,
    const std::string &message) const 
{
    const std::string mode = transport_ ? transport_->mode_name() : "MOCK";
    return utils::make_rejected_result_json(goal, mode, code, message);
}

FeedbackCallback AdapterNode::make_feedback_callback(
    const std::shared_ptr<ExecuteGoalHandle> &goal_handle) {
    // 执行器只提供进度数据，这里绑定当前作业的 Action Goal。
    return [this, goal_handle](
        uint8_t stage,
        uint32_t current_index,
        float progress,
        const std::string &instruction_id,
        const std::string &message) {
        publish_feedback(*goal_handle, stage, current_index, progress, instruction_id, message);
    };
}

void AdapterNode::publish_feedback(
    ExecuteGoalHandle &goal_handle,
    uint8_t stage,
    uint32_t current_index,
    float progress,
    const std::string &instruction_id,
    const std::string &message) {
    auto feedback = std::make_shared<ExecuteBackendJob::Feedback>();
    feedback->stage = stage;
    feedback->current_index = current_index;
    feedback->progress = progress;
    feedback->current_instruction_id = instruction_id;
    feedback->message = message;
    goal_handle.publish_feedback(feedback);
    if (debug_recorder_) {
        debug_recorder_->record_feedback(goal_handle.get_goal_id(), *goal_handle.get_goal(), *feedback);
    }
}

void AdapterNode::finish_action(
    ExecuteGoalHandle &goal_handle,
    const std::shared_ptr<ExecuteBackendJob::Result> &result,
    ExecutionOutcome outcome) {
    const char *action_state = "ABORTED";
    if (outcome == ExecutionOutcome::SUCCEEDED) {
        goal_handle.succeed(result);
        action_state = "SUCCEEDED";
    } else if (outcome == ExecutionOutcome::CANCELED && goal_handle.is_canceling()) {
        goal_handle.canceled(result);
        action_state = "CANCELED";
    } else {
        goal_handle.abort(result);
    }
    if (debug_recorder_) {
        debug_recorder_->record_result(
            goal_handle.get_goal_id(), *goal_handle.get_goal(), *result, action_state);
    }
}

//下面三个是动作的回调函数

rclcpp_action::GoalResponse AdapterNode::goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ExecuteBackendJob::Goal> goal) {

    bool accepted = false;
    const char *reason = "accepted";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_job_) {
            if (current_job_->stop_failed.load()) {
                reason = "stop_unconfirmed";
                RCLCPP_ERROR(
                    get_logger(),
                    "前序任务 %s 停机未确认（KA_STOP_FAILED），拒绝新任务，请先调用 StopBackendJob 确认停机。",
                    current_job_->job_id.c_str());
            } else {
                reason = "busy";
                RCLCPP_WARN(get_logger(), "当前有任务正在执行，拒绝新的任务请求。");
            }
        } else {
            current_job_ = std::make_shared<active_job>(uuid, goal->job_id);
            accepted = true;
        }
    }
    if (debug_recorder_) {
        debug_recorder_->record_goal(uuid, *goal, accepted, reason);
    }
    return accepted ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                    : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse AdapterNode::cancel_callback(
    std::shared_ptr<ExecuteGoalHandle> goal_handle) {
    const auto &uuid = goal_handle->get_goal_id();
    bool accepted = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_job_ && !current_job_->terminal.load() && current_job_->uuid == uuid) {
            RCLCPP_WARN(get_logger(), "收到任务取消请求: %s", current_job_->job_id.c_str());
            current_job_->stop_requested.store(true);
            accepted = true;
        }
    }
    if (debug_recorder_) {
        debug_recorder_->record_cancel(uuid, *goal_handle->get_goal(), accepted);
    }
    return accepted ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
}

void AdapterNode::accepted_callback(std::shared_ptr<ExecuteGoalHandle> goal_handle) {
    
    //回收上一次任务的线程
    if(job_thread_.joinable()){
        job_thread_.join();
    }
    std::shared_ptr<active_job> job;
    // 获取当前任务
    {
        std::lock_guard<std::mutex> lock(mutex_);
        job = current_job_;
    }
    if (!job) {
        RCLCPP_ERROR(get_logger(), "当前任务不存在。");

        auto result = std::make_shared<ExecuteBackendJob::Result>();
        result->success = false;
        result->error_code = to_error_code_string(ErrorCode::KA_PROGRAM_START_FAILED);
        result->message = "当前任务不存在，无法执行。";

        finish_action(*goal_handle, result, ExecutionOutcome::FAILED);
        return;
    }
    // 创建新的工作线程
    job_thread_ = std::thread(
        &AdapterNode::execute_job,
        this,
        job,
        goal_handle
    );
}

void AdapterNode::stop_job_callback(
    const std::shared_ptr<StopBackendJob::Request> request,
    std::shared_ptr<StopBackendJob::Response> response) {
    if (debug_recorder_) {
        debug_recorder_->record_stop_request(*request);
    }
    handle_stop_job(*request, *response);
    if (debug_recorder_) {
        debug_recorder_->record_stop_response(*request, *response);
    }
}

void AdapterNode::handle_stop_job(
    const StopBackendJob::Request &request,
    StopBackendJob::Response &response) {
    std::shared_ptr<active_job> job;
    bool retry_latched_stop = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_job_) {
            response.accepted = false;
            response.message = "当前没有正在执行的任务。";
            return;
        }
        if (!request.job_id.empty() && current_job_->job_id != request.job_id) {
            response.accepted = false;
            response.message = "请求的 job_id (" + request.job_id +
                                ") 与当前任务 (" + current_job_->job_id + ") 不匹配。";
            return;
        }

        if (!current_job_->terminal.load()) {
            // 正在执行中：只登记停止请求，由执行线程独占下发停机，避免跨线程启停竞态
            current_job_->stop_requested.store(true);
            RCLCPP_WARN(get_logger(), "收到 StopBackendJob 停止请求（已受理）: %s",
                        current_job_->job_id.c_str());
            response.accepted = true;
            response.message = "停止请求已受理，正在停止作业。";
            return;
        }

        if (current_job_->stop_failed.load()) {
            // 执行线程已因 KA_STOP_FAILED 退出且锁槽，允许再次调用停机来解锁
            job = current_job_;
            retry_latched_stop = true;
        }
    }

    if (!retry_latched_stop || !job) {
        response.accepted = false;
        response.message = "当前任务已结束。";
        return;
    }

    RCLCPP_WARN(get_logger(), "重试停机确认（KA_STOP_FAILED 锁槽恢复）: %s", job->job_id.c_str());
    const auto stop_res = transport_->stop_program();
    if (stop_res.ok) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_job_ == job) {
            current_job_.reset();
        }
        response.accepted = true;
        response.message = "停机已确认，任务槽已释放。";
    } else {
        response.accepted = false;
        response.message = "停机仍未确认，任务槽保持锁定：" + stop_res.message;
    }
}

void AdapterNode::publish_status() {
    if (!status_pub_ || !transport_) {
        return;
    }

    const KrcStatus krc_status = transport_->read_status();
    std::string active_job_id;
    bool stop_failed_latched = false;
    bool stopping_active = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_job_ && (!current_job_->terminal.load() || current_job_->stop_failed.load())) {
            active_job_id = current_job_->job_id;
            stop_failed_latched = current_job_->stop_failed.load();
            stopping_active = !current_job_->terminal.load() && current_job_->stop_requested.load();
        }
    }

    BackendStatus msg;
    msg.stamp = now();
    msg.backend = "kuka_arctech";
    msg.controller_connected = krc_status.controller_connected;
    msg.program_state = stopping_active ? "STOPPING" : krc_status.program_state;
    msg.weld_state = krc_status.weld_state;
    msg.current_job_id = active_job_id;
    msg.current_program = krc_status.current_program;
    msg.fault_code =
        (stop_failed_latched && krc_status.fault_code.empty())
            ? to_error_code_string(ErrorCode::KA_STOP_FAILED)
            : krc_status.fault_code;
    msg.message =
        stop_failed_latched
            ? ("停机未确认（KA_STOP_FAILED），任务槽已锁定：" + krc_status.message)
            : krc_status.message;
    status_pub_->publish(msg);
    if (debug_recorder_) {
        debug_recorder_->record_status(msg);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(AdapterNode)
