#include "adapter_node.h"
#include "error_code.h"
#include "mock_krc_transport.h"

#include <chrono>
#include <functional>

#include <nlohmann/json.hpp>
#include <rclcpp_components/register_node_macro.hpp>

AdapterNode::AdapterNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("adapter_node", options),
      transport_(std::make_shared<MockKrcTransport>()),
      program_executor_(transport_) {
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

    RCLCPP_INFO(get_logger(), "AdapterNode initialized (mode=%s).", transport_->mode_name().c_str());
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
    RCLCPP_INFO(get_logger(), "开始执行任务: %s", job->job_id.c_str());

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ExecuteBackendJob::Result>();
    result->success = false;
    result->backend_job_id = "b_" + goal->job_id;
    result->artifact_manifest_json = "{}";

    auto make_rejected_result_json = [&](ErrorCode code, const std::string &msg) {
        const std::string mode = transport_ ? transport_->mode_name() : "MOCK";
        nlohmann::json step_results = nlohmann::json::array();
        for (std::size_t i = 0; i < goal->instructions.size(); ++i) {
            const auto &inst = goal->instructions[i];
            step_results.push_back({
                {"index", static_cast<uint32_t>(i)},
                {"instance_id", inst.instance_id},
                {"instruction_id", inst.instruction_id},
                {"state", "SKIPPED"},
                {"payload", nlohmann::json::object()},
                {"error_code", to_error_code_string(code)},
                {"message", msg},
                {"evidence", {{"mode", mode}, {"simulated", mode == "MOCK"}}},
            });
        }
        return nlohmann::json{
            {"contract_version", "1.0"},
            {"job_id", goal->job_id},
            {"trace_id", goal->trace_id},
            {"mode", mode},
            {"step_results", std::move(step_results)},
        }.dump();
    };

    auto publish_feedback = [&](uint8_t stage,
                                uint32_t current_index,
                                float progress,
                                const std::string &instruction_id,
                                const std::string &message) {
        auto fb = std::make_shared<ExecuteBackendJob::Feedback>();
        fb->stage = stage;
        fb->current_index = current_index;
        fb->progress = progress;
        fb->current_instruction_id = instruction_id;
        fb->message = message;
        goal_handle->publish_feedback(fb);
    };

    ExecutionOutcome outcome = ExecutionOutcome::FAILED;
    std::string error_message;
    //检查当前任务是否合法
    publish_feedback(
        static_cast<uint8_t>(ExecutionStage::VALIDATING),
        0,
        0.0f,
        goal->instructions.empty() ? "" : goal->instructions.front().instruction_id,
        "Validating job");

    auto err = job_validator_.validate(*goal, error_message);
    if (err != ErrorCode::OK) {
        RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
        result->error_code = to_error_code_string(err);
        result->message = error_message;
        result->result_json = make_rejected_result_json(err, error_message);
    } else {
        //这里先把任务解析为IR
        KukaArcTechJobIR job_ir;
        err = semantic_mapper_.map(*goal, job_ir, error_message);
        if (err != ErrorCode::OK) {
            RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
            result->error_code = to_error_code_string(err);
            result->message = error_message;
            result->result_json = make_rejected_result_json(err, error_message);
        } else {
            //然后再检查IR是否合法
            err = job_validator_.validate_ir(job_ir, error_message);
            if (err != ErrorCode::OK) {
                RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
                result->error_code = to_error_code_string(err);
                result->message = error_message;
                result->result_json = make_rejected_result_json(err, error_message);
            } else {
                //通过KRC Transport执行作业并收集逐步结果
                const auto exec_res = program_executor_.execute(
                    job_ir,
                    job->stop_requested,
                    publish_feedback);
                outcome = exec_res.outcome;
                result->success = (exec_res.outcome == ExecutionOutcome::SUCCEEDED);
                result->error_code = to_error_code_string(exec_res.error_code);
                result->message = exec_res.message;
                result->backend_job_id = exec_res.backend_job_id;
                result->artifact_manifest_json = exec_res.artifact_manifest_json;
                result->result_json = exec_res.result_json;
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

    if (outcome == ExecutionOutcome::SUCCEEDED) {
        goal_handle->succeed(result);
    } else if (outcome == ExecutionOutcome::CANCELED && goal_handle->is_canceling()) {
        goal_handle->canceled(result);
    } else {
        goal_handle->abort(result);
    }
}

//下面三个是动作的回调函数

rclcpp_action::GoalResponse AdapterNode::goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ExecuteBackendJob::Goal> goal) {

    std::lock_guard<std::mutex> lock(mutex_);
    if (current_job_) {
        if (current_job_->stop_failed.load()) {
            RCLCPP_ERROR(
                get_logger(),
                "前序任务 %s 停机未确认（KA_STOP_FAILED），拒绝新任务，请先调用 StopBackendJob 确认停机。",
                current_job_->job_id.c_str());
        } else {
            RCLCPP_WARN(get_logger(),
                "当前有任务正在执行，拒绝新的任务请求。");
        }
        return rclcpp_action::GoalResponse::REJECT;
    }
    //如果当前没有任务正在执行，则接受新的任务请求
    current_job_ = std::make_shared<active_job>(uuid, goal->job_id);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AdapterNode::cancel_callback(
    std::shared_ptr<ExecuteGoalHandle> goal_handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_job_ && !current_job_->terminal.load() &&
        current_job_->uuid == goal_handle->get_goal_id()) {
        RCLCPP_WARN(get_logger(), "收到任务取消请求: %s", current_job_->job_id.c_str());
        current_job_->stop_requested.store(true);
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    return rclcpp_action::CancelResponse::REJECT;
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

        goal_handle->abort(result);
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
    std::shared_ptr<active_job> job;
    bool retry_latched_stop = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_job_) {
            response->accepted = false;
            response->message = "当前没有正在执行的任务。";
            return;
        }
        if (!request->job_id.empty() && current_job_->job_id != request->job_id) {
            response->accepted = false;
            response->message = "请求的 job_id (" + request->job_id +
                                ") 与当前任务 (" + current_job_->job_id + ") 不匹配。";
            return;
        }

        if (!current_job_->terminal.load()) {
            // 正在执行中：只登记停止请求，由执行线程独占下发停机，避免跨线程启停竞态
            current_job_->stop_requested.store(true);
            RCLCPP_WARN(get_logger(), "收到 StopBackendJob 停止请求（已受理）: %s",
                        current_job_->job_id.c_str());
            response->accepted = true;
            response->message = "停止请求已受理，正在停止作业。";
            return;
        }

        if (current_job_->stop_failed.load()) {
            // 执行线程已因 KA_STOP_FAILED 退出且锁槽，允许再次调用停机来解锁
            job = current_job_;
            retry_latched_stop = true;
        }
    }

    if (!retry_latched_stop || !job) {
        response->accepted = false;
        response->message = "当前任务已结束。";
        return;
    }

    RCLCPP_WARN(get_logger(), "重试停机确认（KA_STOP_FAILED 锁槽恢复）: %s", job->job_id.c_str());
    const auto stop_res = transport_->stop_program();
    if (stop_res.ok) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_job_ == job) {
            current_job_.reset();
        }
        response->accepted = true;
        response->message = "停机已确认，任务槽已释放。";
    } else {
        response->accepted = false;
        response->message = "停机仍未确认，任务槽保持锁定：" + stop_res.message;
    }
}

void AdapterNode::publish_status() {
    if (!status_pub_ || !transport_) {
        return;
    }

    const KrcStatus krc_status = transport_->read_status();
    std::string active_job_id;
    bool stop_failed_latched = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_job_ && (!current_job_->terminal.load() || current_job_->stop_failed.load())) {
            active_job_id = current_job_->job_id;
            stop_failed_latched = current_job_->stop_failed.load();
        }
    }

    BackendStatus msg;
    msg.stamp = now();
    msg.backend = "kuka_arctech";
    msg.controller_connected = krc_status.controller_connected;
    msg.program_state = krc_status.program_state;
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
}

RCLCPP_COMPONENTS_REGISTER_NODE(AdapterNode)
