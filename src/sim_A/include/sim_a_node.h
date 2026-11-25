#ifndef SIM_A_NODE_H
#define SIM_A_NODE_H

#include <memory>
#include <string>

#include <atomic_adapter_interfaces/action/execute_backend_job.hpp>
#include <atomic_adapter_interfaces/msg/atomic_instruction.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class SimANode : public rclcpp::Node
{
public:
    using ExecuteBackendJob = atomic_adapter_interfaces::action::ExecuteBackendJob;
    using AtomicInstruction = atomic_adapter_interfaces::msg::AtomicInstruction;
    using GoalHandleExecute = rclcpp_action::ClientGoalHandle<ExecuteBackendJob>;

    explicit SimANode(const rclcpp::NodeOptions &options);
    ~SimANode() override = default;

private:
    //从config目录加载任务JSON并发送Goal
    void send_goal();

    bool load_goal_from_file(
        const std::string &job_file,
        ExecuteBackendJob::Goal &goal,
        std::string &resolved_path,
        std::string &error_message) const;

    //Action客户端回调
    void goal_response_callback(const GoalHandleExecute::SharedPtr &goal_handle);

    void feedback_callback(
        GoalHandleExecute::SharedPtr,
        const std::shared_ptr<const ExecuteBackendJob::Feedback> feedback);

    void result_callback(const GoalHandleExecute::WrappedResult &result);

    void maybe_shutdown();

    //参数配置
    std::string job_file_;
    std::string action_name_;
    double wait_server_sec_{5.0};
    double cancel_after_sec_{0.0};
    bool auto_shutdown_{true};

    //ROS2客户端与定时器
    rclcpp_action::Client<ExecuteBackendJob>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::TimerBase::SharedPtr cancel_timer_;
    GoalHandleExecute::SharedPtr active_goal_handle_;
};

#endif // SIM_A_NODE_H
