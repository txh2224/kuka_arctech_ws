#include "job_validator.h"

#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <utility>
#include <variant>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

//有些数据需要从string中解析出来以后才能判断是否合法,下面这个函数是检查一些基本字段是否合法的函数,如果不合法就返回错误码,并且把错误信息写入error_message中
ErrorCode JobValidator::validate(
    const atomic_adapter_interfaces::action::ExecuteBackendJob::Goal &goal,
    std::string &error_message) const {
    error_message.clear();

    if (goal.job_id.empty()) {
        error_message = "job_id 不能为空。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (goal.trace_id.empty()) {
        error_message = "trace_id 不能为空。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (goal.backend != "kuka_arctech") {
        error_message = "不支持的后端类型：" + goal.backend;
        return ErrorCode::KA_JOB_INVALID;
    }
    if (goal.instructions.empty()) {
        error_message = "原子指令序列不能为空。";
        return ErrorCode::KA_JOB_INVALID;
    }

    return ErrorCode::OK;
}

//这里的检查是在原子指令被解析为IR之后的检查
//这部分给哈基米写了，改的太多了

ErrorCode JobValidator::validate_program_name(
    const std::string &program_name,
    std::string &error_message) const {
    if (program_name.empty()) {
        error_message = "program_name 不能为空。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (program_name.size() > config_.max_program_name_length) {
        error_message = "program_name 长度超过限制。";
        return ErrorCode::KA_JOB_INVALID;
    }
    const unsigned char first = static_cast<unsigned char>(program_name.front());
    if (!std::isalpha(first)) {
        error_message = "program_name 必须以字母开头：" + program_name;
        return ErrorCode::KA_JOB_INVALID;
    }
    for (char ch : program_name) {
        const unsigned char uch = static_cast<unsigned char>(ch);
        if (!std::isalnum(uch) && ch != '_') {
            error_message = "program_name 包含非法字符（仅允许字母、数字和下划线）：" + program_name;
            return ErrorCode::KA_JOB_INVALID;
        }
    }
    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_pose(
    const KukaPose &pose,
    const std::string &label,
    std::string &error_message) const {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z) ||
        !std::isfinite(pose.a) || !std::isfinite(pose.b) || !std::isfinite(pose.c)) {
        error_message = label + " 位姿包含未赋值或非法浮点值（NaN 或 Inf）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (pose.x < config_.min_x_mm || pose.x > config_.max_x_mm ||
        pose.y < config_.min_y_mm || pose.y > config_.max_y_mm ||
        pose.z < config_.min_z_mm || pose.z > config_.max_z_mm) {
        error_message = label + " 位置超出 KUKA 允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (pose.a < config_.min_angle_deg || pose.a > config_.max_angle_deg ||
        pose.b < config_.min_angle_deg || pose.b > config_.max_angle_deg ||
        pose.c < config_.min_angle_deg || pose.c > config_.max_angle_deg) {
        error_message = label + " 姿态角超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_step(
    const MoveJointStep &step,
    std::string &error_message) const {
    for (std::size_t i = 0; i < step.joint_names.size(); ++i) {
        if (step.joint_names[i].empty()) {
            error_message = "move_joint 第 " + std::to_string(i) + " 个关节名不能为空。";
            return ErrorCode::KA_JOB_INVALID;
        }
        const double q = step.target_rad[i];
        if (!std::isfinite(q)) {
            error_message = "move_joint 第 " + std::to_string(i) + " 个关节目标角未赋值或非法。";
            return ErrorCode::KA_JOB_INVALID;
        }
        if (q < config_.min_joint_rad || q > config_.max_joint_rad) {
            error_message = "move_joint 第 " + std::to_string(i) + " 个关节目标角超出允许范围。";
            return ErrorCode::KA_JOB_INVALID;
        }
    }
    if (!std::isfinite(step.max_velocity_rad_s) || step.max_velocity_rad_s <= 0.0 ||
        step.max_velocity_rad_s > config_.max_joint_velocity_rad_s) {
        error_message = "move_joint 关节速度未赋值或超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!std::isfinite(step.max_acceleration_rad_s2) || step.max_acceleration_rad_s2 <= 0.0 ||
        step.max_acceleration_rad_s2 > config_.max_joint_acceleration_rad_s2) {
        error_message = "move_joint 关节加速度未赋值或超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_step(
    const MoveLinearStep &step,
    std::string &error_message) const {
    if (const auto err = validate_pose(step.target_pose, "move_linear", error_message);
        err != ErrorCode::OK) {
        return err;
    }
    if (!std::isfinite(step.speed_m_s) || step.speed_m_s <= 0.0 ||
        step.speed_m_s > config_.max_lin_speed_m_s) {
        error_message = "move_linear 速度未赋值或超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!std::isfinite(step.acceleration_m_s2) || step.acceleration_m_s2 <= 0.0 ||
        step.acceleration_m_s2 > config_.max_lin_acceleration_m_s2) {
        error_message = "move_linear 加速度未赋值或超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_step(
    const FollowPathStep &step,
    std::string &error_message) const {
    if (step.waypoints.empty()) {
        error_message = "缺少焊接路径点（follow_path.waypoints 为空）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (step.waypoints.size() < 2) {
        error_message = "焊接路径至少需要包含起点和终点（至少 2 个点）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (step.waypoints.size() > config_.max_weld_path_points) {
        error_message = "焊接路径点数超出 V1 配置上限。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!std::isfinite(step.speed_m_s) || step.speed_m_s <= 0.0 ||
        step.speed_m_s > config_.max_weld_speed_m_s) {
        error_message = "follow_path 焊接速度未赋值或超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    for (std::size_t i = 0; i < step.waypoints.size(); ++i) {
        if (const auto err = validate_pose(
                step.waypoints[i],
                "follow_path[" + std::to_string(i) + "]",
                error_message);
            err != ErrorCode::OK) {
            return err;
        }
    }
    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_step(
    const RecipeSelectStep &step,
    std::string &error_message) const {
    if (step.program_id.empty()) {
        error_message = "welder.recipe.select 缺少 program_id。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (step.weld_data_ref.empty()) {
        error_message = "welder.recipe.select 缺少映射后的 WDAT 配方名（weld_data_ref）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!config_.wdat_allowlist.empty() &&
        config_.wdat_allowlist.find(step.weld_data_ref) == config_.wdat_allowlist.end()) {
        error_message = "焊接配方不在允许的 WDAT 白名单内：" + step.weld_data_ref;
        return ErrorCode::KA_JOB_INVALID;
    }
    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_step(
    const ArcStartStep &step,
    std::string &error_message) const {
    if (step.start_profile_id.empty()) {
        error_message = "welder.arc.start 缺少 start_profile_id。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (step.lease_ms == 0) {
        error_message = "welder.arc.start 的 lease_ms 必须大于 0。";
        return ErrorCode::KA_JOB_INVALID;
    }
    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_step(
    const ArcStopStep &step,
    std::string &error_message) const {
    if (step.crater_profile_id.empty()) {
        error_message = "welder.arc.stop 缺少 crater_profile_id。";
        return ErrorCode::KA_JOB_INVALID;
    }
    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_step(
    const MeasureProcessStep &step,
    std::string &error_message) const {
    if (step.channels.empty()) {
        error_message = "welder.measure.get_process 缺少 channels。";
        return ErrorCode::KA_JOB_INVALID;
    }
    for (const auto &ch : step.channels) {
        if (ch.empty()) {
            error_message = "welder.measure.get_process 包含空通道名。";
            return ErrorCode::KA_JOB_INVALID;
        }
        if (!config_.allowed_measure_channels.empty() &&
            config_.allowed_measure_channels.find(ch) == config_.allowed_measure_channels.end()) {
            error_message = "不支持的过程量采样通道：" + ch;
            return ErrorCode::KA_JOB_INVALID;
        }
    }
    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_sequence(
    const struct KukaArcTechJobIR &job_ir,
    std::string &error_message) const {
    bool has_recipe = false;
    bool arc_active = false;

    for (const auto &step : job_ir.steps) {
        if (std::holds_alternative<RecipeSelectStep>(step.payload)) {
            has_recipe = true;
        } else if (std::holds_alternative<ArcStartStep>(step.payload)) {
            if (!has_recipe) {
                error_message = "序列中在起弧（ARC ON）前缺少焊接配方选择（welder.recipe.select）。";
                return ErrorCode::KA_JOB_INVALID;
            }
            if (arc_active) {
                error_message = "序列中出现重复起弧（ARC ON），前一次电弧尚未关闭。";
                return ErrorCode::KA_JOB_INVALID;
            }
            arc_active = true;
        } else if (std::holds_alternative<ArcStopStep>(step.payload)) {
            if (!arc_active) {
                error_message = "序列中收弧（ARC OFF）出现在起弧（ARC ON）之前或缺少配对起弧。";
                return ErrorCode::KA_JOB_INVALID;
            }
            arc_active = false;
        }
    }

    if (arc_active) {
        error_message = "序列中起弧（ARC ON）与收弧（ARC OFF）不成对，缺少收弧指令。";
        return ErrorCode::KA_JOB_INVALID;
    }

    return ErrorCode::OK;
}

ErrorCode JobValidator::validate_ir(
    const struct KukaArcTechJobIR &job_ir,
    std::string &error_message) const {
    error_message.clear();

    // 1. 基础元信息
    if (job_ir.job_id.empty()) {
        error_message = "IR 中 job_id 不能为空。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.trace_id.empty()) {
        error_message = "IR 中 trace_id 不能为空。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!job_ir.program_name.empty()) {
        if (const auto err = validate_program_name(job_ir.program_name, error_message);
            err != ErrorCode::OK) {
            return err;
        }
    }
    if (job_ir.tool_id >= 0 && job_ir.tool_id > config_.max_tool_id) {
        error_message = "tool_id 超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.base_id >= 0 && job_ir.base_id > config_.max_base_id) {
        error_message = "base_id 超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.steps.empty()) {
        error_message = "IR 中 steps 不能为空。";
        return ErrorCode::KA_JOB_INVALID;
    }

    // 2. 逐条检查每一步的元信息与参数，同时统计作业包含的运动/焊接类型
    bool has_cartesian_step = false;
    bool has_arc_start = false;
    for (std::size_t i = 0; i < job_ir.steps.size(); ++i) {
        const auto &step = job_ir.steps[i];
        if (step.index != i) {
            error_message = "步骤索引不连续：期望 " + std::to_string(i) +
                            "，实际为 " + std::to_string(step.index) + "。";
            return ErrorCode::KA_JOB_INVALID;
        }
        if (step.instruction_id.empty() || step.instance_id.empty() ||
            step.capability_id.empty()) {
            error_message = "第 " + std::to_string(i) +
                            " 步缺少 instruction_id / instance_id / capability_id。";
            return ErrorCode::KA_JOB_INVALID;
        }

        if (std::holds_alternative<MoveLinearStep>(step.payload) ||
            std::holds_alternative<FollowPathStep>(step.payload)) {
            has_cartesian_step = true;
        } else if (std::holds_alternative<ArcStartStep>(step.payload)) {
            has_arc_start = true;
        }

        const auto err = std::visit(
            [&](const auto &payload) { return validate_step(payload, error_message); },
            step.payload);
        if (err != ErrorCode::OK) {
            return err;
        }
    }

    // 3. 按作业实际包含的步骤校验顶层 tool_id / base_id / seam_id
    if (has_cartesian_step) {
        if (job_ir.tool_id < 0 || job_ir.tool_id > config_.max_tool_id) {
            error_message = "包含笛卡尔运动但 tool_id 未赋值或超出允许范围。";
            return ErrorCode::KA_JOB_INVALID;
        }
        if (job_ir.base_id < 0 || job_ir.base_id > config_.max_base_id) {
            error_message = "包含笛卡尔运动但 base_id 未赋值或超出允许范围。";
            return ErrorCode::KA_JOB_INVALID;
        }
    }
    if (has_arc_start && !job_ir.dry_run && job_ir.seam_id == 0) {
        error_message = "实焊任务包含起弧但 seam_id 未赋值（必须大于 0）。";
        return ErrorCode::KA_JOB_INVALID;
    }

    // 4. 多步序列的时序与工艺检查（N=1 单条指令跳过多步配对约束）
    if (job_ir.steps.size() > 1) {
        if (const auto err = validate_sequence(job_ir, error_message);
            err != ErrorCode::OK) {
            return err;
        }
    }

    return ErrorCode::OK;
}

//读取配置文件，覆盖默认参数，可能以后上真机调试时才知道一些范围
void JobValidator::read_config() {
    std::filesystem::path config_path;
    try {
        const auto share_dir =
            ament_index_cpp::get_package_share_directory("kuka_arctech_adapter");
        config_path = std::filesystem::path(share_dir) / "config" / "validator.json";
    } catch (const std::exception &) {
        config_path = std::filesystem::path("config") / "validator.json";
    }

    if (!std::filesystem::exists(config_path)) {
        std::cerr << "[JobValidator] 未找到配置文件，使用默认参数："
                  << config_path << std::endl;
        return;
    }

    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        std::cerr << "[JobValidator] 无法打开配置文件，使用默认参数："
                  << config_path << std::endl;
        return;
    }

    try {
        nlohmann::json j;
        ifs >> j;

        if (j.contains("max_program_name_length")) {
            config_.max_program_name_length =
                j.at("max_program_name_length").get<std::size_t>();
        }
        if (j.contains("max_tool_id")) {
            config_.max_tool_id = j.at("max_tool_id").get<int>();
        }
        if (j.contains("max_base_id")) {
            config_.max_base_id = j.at("max_base_id").get<int>();
        }
        if (j.contains("max_weld_path_points")) {
            config_.max_weld_path_points =
                j.at("max_weld_path_points").get<std::size_t>();
        }
        if (j.contains("max_ptp_speed_percent")) {
            config_.max_ptp_speed_percent =
                j.at("max_ptp_speed_percent").get<double>();
        }
        if (j.contains("max_joint_velocity_rad_s")) {
            config_.max_joint_velocity_rad_s =
                j.at("max_joint_velocity_rad_s").get<double>();
        }
        if (j.contains("max_joint_acceleration_rad_s2")) {
            config_.max_joint_acceleration_rad_s2 =
                j.at("max_joint_acceleration_rad_s2").get<double>();
        }
        if (j.contains("max_lin_speed_m_s")) {
            config_.max_lin_speed_m_s = j.at("max_lin_speed_m_s").get<double>();
        }
        if (j.contains("max_lin_acceleration_m_s2")) {
            config_.max_lin_acceleration_m_s2 =
                j.at("max_lin_acceleration_m_s2").get<double>();
        }
        if (j.contains("max_weld_speed_m_s")) {
            config_.max_weld_speed_m_s = j.at("max_weld_speed_m_s").get<double>();
        }
        if (j.contains("min_joint_rad")) {
            config_.min_joint_rad = j.at("min_joint_rad").get<double>();
        }
        if (j.contains("max_joint_rad")) {
            config_.max_joint_rad = j.at("max_joint_rad").get<double>();
        }
        if (j.contains("min_x_mm")) {
            config_.min_x_mm = j.at("min_x_mm").get<double>();
        }
        if (j.contains("max_x_mm")) {
            config_.max_x_mm = j.at("max_x_mm").get<double>();
        }
        if (j.contains("min_y_mm")) {
            config_.min_y_mm = j.at("min_y_mm").get<double>();
        }
        if (j.contains("max_y_mm")) {
            config_.max_y_mm = j.at("max_y_mm").get<double>();
        }
        if (j.contains("min_z_mm")) {
            config_.min_z_mm = j.at("min_z_mm").get<double>();
        }
        if (j.contains("max_z_mm")) {
            config_.max_z_mm = j.at("max_z_mm").get<double>();
        }
        if (j.contains("min_angle_deg")) {
            config_.min_angle_deg = j.at("min_angle_deg").get<double>();
        }
        if (j.contains("max_angle_deg")) {
            config_.max_angle_deg = j.at("max_angle_deg").get<double>();
        }
        if (j.contains("wdat_allowlist") && j.at("wdat_allowlist").is_array()) {
            config_.wdat_allowlist.clear();
            for (const auto &item : j.at("wdat_allowlist")) {
                config_.wdat_allowlist.insert(item.get<std::string>());
            }
        }
        if (j.contains("allowed_measure_channels") &&
            j.at("allowed_measure_channels").is_array()) {
            config_.allowed_measure_channels.clear();
            for (const auto &item : j.at("allowed_measure_channels")) {
                config_.allowed_measure_channels.insert(item.get<std::string>());
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "[JobValidator] 解析配置文件失败，保留默认参数："
                  << e.what() << std::endl;
    }
}
