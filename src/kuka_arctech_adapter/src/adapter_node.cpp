#include "adapter_node.h"

#include <functional>

#include <rclcpp_components/register_node_macro.hpp>

AdapterNode::AdapterNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("adapter_node", options) {

    //这里创建动作Server，绑定回调函数
    execute_action_ = rclcpp_action::create_server<ExecuteBackendJob>(
        this,
        "execute_backend_job",
        std::bind(&AdapterNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&AdapterNode::cancel_callback, this, std::placeholders::_1),
        std::bind(&AdapterNode::accepted_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "AdapterNode initialized.");
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

    std::string error_message;
    //检查当前任务是否合法
    if (!check_job(*goal, error_message)) {
        RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
        result->error_code = "KA_JOB_INVALID";
        result->message = error_message;
    } else {
        result->error_code = "NOT_IMPLEMENTED";
        result->message = "KUKA ArcTech 后端执行尚未接入。";
    }

    // 先释放任务槽，客户端收到结果后就可以提交下一单。
    {
        std::lock_guard<std::mutex> lock(mutex_);
        job->terminal = true;
        current_job_.reset();
    }
    goal_handle->abort(result);
}

bool AdapterNode::check_job(const ExecuteBackendJob::Goal &goal, std::string &error_message) {
    //检查当前任务是否合法
    error_message.clear();

    if(goal.job_id.empty()) {
        error_message = "job_id 不能为空。";
        return false;
    }
    if(goal.trace_id.empty()) {
        error_message = "trace_id 不能为空。";
        return false;
    }
    if(goal.backend != "kuka_arctech") {
        error_message = "不支持的后端类型：" + goal.backend;
        return false;
    }
    if(goal.instructions.empty()) {
        error_message = "原子指令序列不能为空。";
        return false;
    }

    return true;
}

//下面三个是动作的回调函数

rclcpp_action::GoalResponse AdapterNode::goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ExecuteBackendJob::Goal> goal) {

    std::lock_guard<std::mutex> lock(mutex_);
    if (current_job_ ) {
        RCLCPP_WARN(get_logger(),
            "当前有任务正在执行，拒绝新的任务请求。");
        return rclcpp_action::GoalResponse::REJECT;
    }
    //如果当前没有任务正在执行，则接受新的任务请求
    current_job_ = std::make_shared<active_job>(uuid, goal->job_id);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AdapterNode::cancel_callback(
    std::shared_ptr<ExecuteGoalHandle>) {
    


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
        result->error_code = "KA_PROGRAM_START_FAILED";
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





RCLCPP_COMPONENTS_REGISTER_NODE(AdapterNode)
