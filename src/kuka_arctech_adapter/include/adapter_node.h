#ifndef ADAPTER_NODE_H
#define ADAPTER_NODE_H

#include "job_validator.h"
#include "krc_transport.h"
#include "program_executor.h"
#include "semantic_mapper.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <atomic_adapter_interfaces/action/execute_backend_job.hpp>
#include <atomic_adapter_interfaces/msg/backend_status.hpp>
#include <atomic_adapter_interfaces/srv/stop_backend_job.hpp>
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
    std::atomic<bool> stop_requested{false};//标记是否请求停止
    std::atomic<bool> terminal{false};//标记是否已经终止
    std::atomic<bool> stop_failed{false};//标记是否因停机失败锁槽，需再次确认停机后释放

};

class AdapterNode : public rclcpp::Node 
{
public:
    //tmd这手册里的名字怎么起的这么长
    using ExecuteBackendJob = atomic_adapter_interfaces::action::ExecuteBackendJob;
    using ExecuteGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteBackendJob>;
    using BackendStatus = atomic_adapter_interfaces::msg::BackendStatus;
    using StopBackendJob = atomic_adapter_interfaces::srv::StopBackendJob;
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

    //停止服务回调与周期状态发布
    void stop_job_callback(
        const std::shared_ptr<StopBackendJob::Request> request,
        std::shared_ptr<StopBackendJob::Response> response);

    void publish_status();

    //成员变量

    JobValidator job_validator_;//检查任务是否合法的对象
    SemanticMapper semantic_mapper_;//将Goal解析为IR的对象
    std::shared_ptr<IKrcTransport> transport_;//底层KRC通信抽象（默认Mock）
    ProgramExecutor program_executor_;//作业执行器

    //这里定义动作的Server指针
    rclcpp_action::Server<ExecuteBackendJob>::SharedPtr execute_action_;
    rclcpp::Publisher<BackendStatus>::SharedPtr status_pub_;
    rclcpp::Service<StopBackendJob>::SharedPtr stop_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::mutex mutex_; //保护共享资源的互斥锁

    std::shared_ptr<active_job> current_job_; //当前正在执行的任务

    std::thread job_thread_; //执行任务的线程
};

#endif // ADAPTER_NODE_H
