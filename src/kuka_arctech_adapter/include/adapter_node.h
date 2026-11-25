#ifndef ADAPTER_NODE_H
#define ADAPTER_NODE_H

#include "job_validator.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <atomic_adapter_interfaces/action/execute_backend_job.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class active_job
{//从师兄那里学来的方法,这样可以直接记录当前正在执行的任务的uuid和job_id,方便后续的处理
public:
    active_job(const rclcpp_action::GoalUUID &uuid, const std::string &job_id)
        : uuid(uuid), job_id(job_id) {}
    ~active_job() = default;

    const rclcpp_action::GoalUUID uuid;
    const std::string job_id;
    std::atomic <bool> stop_requested{false};//标记是否请求停止
    bool terminal{false};//标记是否已经终止

};

class AdapterNode : public rclcpp::Node 
{
public:
    //tmd这手册里的名字怎么起的这么长
    using ExecuteBackendJob = atomic_adapter_interfaces::action::ExecuteBackendJob;
    using ExecuteGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteBackendJob>;
    explicit AdapterNode(const rclcpp::NodeOptions &options);
    ~AdapterNode() override;

    void execute_job(//执行任务的函数，需要单独开线程来执行，避免阻塞动作的回调线程
        std::shared_ptr<active_job> job,
        std::shared_ptr<ExecuteGoalHandle> goal_handle
    );

private:
    //下面是动作的三个回调函数
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ExecuteBackendJob::Goal> goal);

    rclcpp_action::CancelResponse cancel_callback(std::shared_ptr<ExecuteGoalHandle>);

    void accepted_callback(std::shared_ptr<ExecuteGoalHandle> goal_handle);
    

    //成员变量

    JobValidator job_validator_;//检查任务是否合法的对象

    //这里定义动作的Server指针
    rclcpp_action::Server<ExecuteBackendJob>::SharedPtr execute_action_;

    std::mutex mutex_; //保护共享资源的互斥锁

    std::shared_ptr<active_job> current_job_; //当前正在执行的任务

    std::thread job_thread_; //执行任务的线程
};

#endif // ADAPTER_NODE_H
