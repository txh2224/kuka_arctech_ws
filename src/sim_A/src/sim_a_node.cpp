#include "sim_a_node.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace {

std::string json_field_to_string(const nlohmann::json &obj, const char *key) {
    if (!obj.contains(key) || obj.at(key).is_null()) {
        return "{}";
    }
    const auto &val = obj.at(key);
    if (val.is_string()) {
        return val.get<std::string>();
    }
    return val.dump();
}

const char *result_code_to_string(rclcpp_action::ResultCode code) {
    switch (code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return "SUCCEEDED";
        case rclcpp_action::ResultCode::ABORTED:
            return "ABORTED";
        case rclcpp_action::ResultCode::CANCELED:
            return "CANCELED";
        default:
            return "UNKNOWN";
    }
}

}  // namespace

SimANode::SimANode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("sim_a_node", options) {
    job_file_ = declare_parameter<std::string>("job_file", "job_8steps_demo.json");
    action_name_ = declare_parameter<std::string>(
        "action_name", "/adapter/kuka_arctech/execute_job");
    wait_server_sec_ = declare_parameter<double>("wait_server_sec", 5.0);
    cancel_after_sec_ = declare_parameter<double>("cancel_after_sec", 0.0);
    auto_shutdown_ = declare_parameter<bool>("auto_shutdown", true);

    action_client_ = rclcpp_action::create_client<ExecuteBackendJob>(this, action_name_);

    //节点spin起来之后再触发发送，避免阻塞构造函数
    start_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SimANode::send_goal, this));
}

bool SimANode::load_goal_from_file(
    const std::string &job_file,
    ExecuteBackendJob::Goal &goal,
    std::string &resolved_path,
    std::string &error_message) const {
    namespace fs = std::filesystem;

    fs::path path(job_file);
    if (!fs::exists(path)) {
        try {
            const fs::path share_dir = ament_index_cpp::get_package_share_directory("sim_A");
            const fs::path candidate = share_dir / "config" / job_file;
            if (fs::exists(candidate)) {
                path = candidate;
            }
        } catch (const std::exception &e) {
            error_message = std::string("定位 sim_A share 目录失败: ") + e.what();
            return false;
        }
    }

    if (!fs::exists(path)) {
        error_message = "找不到任务配置文件: " + job_file;
        return false;
    }

    resolved_path = fs::absolute(path).string();
    std::ifstream ifs(resolved_path);
    if (!ifs.is_open()) {
        error_message = "无法打开任务配置文件: " + resolved_path;
        return false;
    }

    nlohmann::json doc;
    try {
        ifs >> doc;
    } catch (const std::exception &e) {
        error_message = std::string("解析 JSON 失败: ") + e.what();
        return false;
    }

    if (!doc.is_object()) {
        error_message = "任务配置根节点必须是 JSON 对象。";
        return false;
    }

    try {
        goal.job_id = doc.value("job_id", "");
        goal.trace_id = doc.value("trace_id", "");
        goal.backend = doc.value("backend", "kuka_arctech");
        goal.dry_run = doc.value("dry_run", true);

        goal.instructions.clear();
        if (doc.contains("instructions")) {
            if (!doc.at("instructions").is_array()) {
                error_message = "instructions 字段必须是数组。";
                return false;
            }
            for (const auto &item : doc.at("instructions")) {
                if (!item.is_object()) {
                    error_message = "instructions 中的每一项都必须是 JSON 对象。";
                    return false;
                }
                AtomicInstruction inst;
                inst.schema_version = item.value("schema_version", "1.0");
                inst.instruction_id = item.value("instruction_id", "");
                inst.instance_id = item.value("instance_id", "");
                inst.correlation_id = item.value("correlation_id", "");
                inst.target_device = item.value("target_device", "");
                inst.capability_id = item.value("capability_id", "");
                inst.action = item.value("action", "");
                inst.object_json = json_field_to_string(item, "object_json");
                inst.parameter_json = json_field_to_string(item, "parameter_json");
                inst.constraint_json = json_field_to_string(item, "constraint_json");
                inst.safety_json = json_field_to_string(item, "safety_json");
                inst.result_contract_json = json_field_to_string(item, "result_contract_json");
                inst.timeout_ms = item.value("timeout_ms", static_cast<uint32_t>(0));
                inst.priority = item.value("priority", static_cast<uint8_t>(5));
                inst.failure_policy = item.value("failure_policy", "ABORT_ON_FAILURE");
                goal.instructions.push_back(std::move(inst));
            }
        }
    } catch (const std::exception &e) {
        error_message = std::string("构造 ExecuteBackendJob Goal 失败: ") + e.what();
        return false;
    }

    return true;
}

void SimANode::send_goal() {
    if (start_timer_) {
        start_timer_->cancel();
    }

    ExecuteBackendJob::Goal goal;
    std::string resolved_path;
    std::string err_msg;
    if (!load_goal_from_file(job_file_, goal, resolved_path, err_msg)) {
        RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
        maybe_shutdown();
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "已加载任务配置: %s (job_id=%s, dry_run=%s, instructions=%zu)",
        resolved_path.c_str(),
        goal.job_id.c_str(),
        goal.dry_run ? "true" : "false",
        goal.instructions.size());

    RCLCPP_INFO(get_logger(), "等待 Action Server 上线: %s ...", action_name_.c_str());
    const auto wait_ms = std::chrono::milliseconds(
        static_cast<int64_t>(wait_server_sec_ * 1000.0));
    if (!action_client_->wait_for_action_server(wait_ms)) {
        RCLCPP_ERROR(
            get_logger(),
            "等待超时 (%.1fs)，未发现 Action Server: %s",
            wait_server_sec_,
            action_name_.c_str());
        maybe_shutdown();
        return;
    }

    rclcpp_action::Client<ExecuteBackendJob>::SendGoalOptions options;
    options.goal_response_callback = std::bind(
        &SimANode::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback = std::bind(
        &SimANode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(
        &SimANode::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "发送 Goal: job_id=%s", goal.job_id.c_str());
    action_client_->async_send_goal(goal, options);
}

void SimANode::goal_response_callback(const GoalHandleExecute::SharedPtr &goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal 被服务端拒绝（REJECTED）。");
        maybe_shutdown();
        return;
    }

    active_goal_handle_ = goal_handle;
    RCLCPP_INFO(get_logger(), "Goal 已被服务端接收（ACCEPTED），等待执行结果...");

    if (cancel_after_sec_ > 0.0) {
        const auto cancel_ms = std::chrono::milliseconds(
            static_cast<int64_t>(cancel_after_sec_ * 1000.0));
        cancel_timer_ = create_wall_timer(cancel_ms, [this]() {
            if (cancel_timer_) {
                cancel_timer_->cancel();
            }
            if (active_goal_handle_) {
                RCLCPP_WARN(
                    get_logger(),
                    "已过 %.2fs，主动发送 Cancel 请求...",
                    cancel_after_sec_);
                action_client_->async_cancel_goal(active_goal_handle_);
            }
        });
    }
}

void SimANode::feedback_callback(
    GoalHandleExecute::SharedPtr,
    const std::shared_ptr<const ExecuteBackendJob::Feedback> feedback) {
    RCLCPP_INFO(
        get_logger(),
        "[Feedback] stage=%u index=%u progress=%.1f%% inst=%s msg='%s'",
        feedback->stage,
        feedback->current_index,
        feedback->progress * 100.0f,
        feedback->current_instruction_id.c_str(),
        feedback->message.c_str());
}

void SimANode::result_callback(const GoalHandleExecute::WrappedResult &result) {
    if (cancel_timer_) {
        cancel_timer_->cancel();
    }
    active_goal_handle_.reset();

    if (!result.result) {
        RCLCPP_ERROR(
            get_logger(),
            "[Result] status=%s，未收到结果载荷。",
            result_code_to_string(result.code));
        maybe_shutdown();
        return;
    }

    const auto &res = *result.result;
    RCLCPP_INFO(
        get_logger(),
        "[Result] status=%s success=%s error_code='%s' backend_job_id='%s' message='%s'",
        result_code_to_string(result.code),
        res.success ? "true" : "false",
        res.error_code.c_str(),
        res.backend_job_id.c_str(),
        res.message.c_str());

    if (!res.artifact_manifest_json.empty() && res.artifact_manifest_json != "{}") {
        try {
            const auto manifest = nlohmann::json::parse(res.artifact_manifest_json);
            std::cout << "=== artifact_manifest_json ===\n" << manifest.dump(2) << std::endl;
        } catch (const std::exception &) {
            std::cout << "=== artifact_manifest_json ===\n"
                      << res.artifact_manifest_json << std::endl;
        }
    }

    try {
        const auto parsed = nlohmann::json::parse(res.result_json);
        std::cout << "=== result_json ===\n" << parsed.dump(2) << std::endl;
    } catch (const std::exception &) {
        std::cout << res.result_json << std::endl;
    }

    maybe_shutdown();
}

void SimANode::maybe_shutdown() {
    if (auto_shutdown_ && rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimANode)
