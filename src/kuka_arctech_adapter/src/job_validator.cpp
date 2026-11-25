#include "job_validator.h"

#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <utility>

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
        error_message = label + " 位姿包含非法浮点值（NaN 或 Inf）。";
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

ErrorCode JobValidator::validate_ir(
    const struct KukaArcTechJobIR &job_ir,
    std::string &error_message) const {
    error_message.clear();

    //基础信息
    if (job_ir.job_id.empty()) {
        error_message = "IR 中 job_id 不能为空。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.trace_id.empty()) {
        error_message = "IR 中 trace_id 不能为空。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (const auto err = validate_program_name(job_ir.program_name, error_message);
        err != ErrorCode::OK) {
        return err;
    }

    //坐标系、焊缝号与 WDAT 配方检查
    if (job_ir.tool_id < 0 || job_ir.tool_id > config_.max_tool_id) {
        error_message = "tool_id 未赋值或超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.base_id < 0 || job_ir.base_id > config_.max_base_id) {
        error_message = "base_id 未赋值或超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.seam_id == 0) {
        error_message = "seam_id 未赋值（必须大于 0）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!job_ir.dry_run && job_ir.weld_data_ref.empty()) {
        error_message = "实焊任务缺少焊接配方（weld_data_ref / WDAT）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!job_ir.weld_data_ref.empty() && !config_.wdat_allowlist.empty() &&
        config_.wdat_allowlist.find(job_ir.weld_data_ref) == config_.wdat_allowlist.end()) {
        error_message = "焊接配方不在允许的 WDAT 白名单内：" + job_ir.weld_data_ref;
        return ErrorCode::KA_JOB_INVALID;
    }

    //接近段（approach）检查
    if (job_ir.approach.type != MotionType::PTP &&
        job_ir.approach.type != MotionType::LIN) {
        error_message = "approach 运动类型未赋值或不支持（V1 仅允许 PTP 或 LIN）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!std::isfinite(job_ir.approach.speed) || job_ir.approach.speed < 0.0) {
        error_message = "approach 速度未赋值或为非法值。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.approach.speed == 0.0) {
        error_message = "approach 速度必须大于 0。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.approach.type == MotionType::PTP &&
        job_ir.approach.speed > config_.max_ptp_speed_percent) {
        error_message = "approach PTP 速度超出允许百分比范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.approach.type == MotionType::LIN &&
        job_ir.approach.speed > config_.max_lin_speed_m_s) {
        error_message = "approach LIN 速度超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (const auto err = validate_pose(job_ir.approach.pose, "approach", error_message);
        err != ErrorCode::OK) {
        return err;
    }

    //焊接路径（weld_path）检查
    if (job_ir.weld_path.type != WeldPathType::LIN_POLYLINE) {
        error_message = "weld_path 类型未赋值或不支持（V1 仅支持 LIN_POLYLINE）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.weld_path.points.empty()) {
        error_message = "缺少焊接路径点（weld_path.points 为空）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.weld_path.points.size() < 2) {
        error_message = "焊接路径至少需要包含起点和终点（至少 2 个点）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.weld_path.points.size() > config_.max_weld_path_points) {
        error_message = "焊接路径点数超出 V1 配置上限。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!std::isfinite(job_ir.weld_path.speed) || job_ir.weld_path.speed < 0.0) {
        error_message = "weld_path 速度未赋值或为非法值。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.weld_path.speed == 0.0 ||
        job_ir.weld_path.speed > config_.max_weld_speed_m_s) {
        error_message = "weld_path 焊接速度超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    for (std::size_t i = 0; i < job_ir.weld_path.points.size(); ++i) {
        if (const auto err = validate_pose(
                job_ir.weld_path.points[i],
                "weld_path[" + std::to_string(i) + "]",
                error_message);
            err != ErrorCode::OK) {
            return err;
        }
    }

    //撤离段（retract）检查
    if (job_ir.retract.type != MotionType::LIN) {
        error_message = "retract 运动类型非法（V1 仅允许 LIN）。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (!std::isfinite(job_ir.retract.speed) || job_ir.retract.speed < 0.0) {
        error_message = "retract 速度未赋值或为非法值。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.retract.speed == 0.0 ||
        job_ir.retract.speed > config_.max_lin_speed_m_s) {
        error_message = "retract 速度超出允许范围。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (const auto err = validate_pose(job_ir.retract.pose, "retract", error_message);
        err != ErrorCode::OK) {
        return err;
    }

    //起弧/收弧与 dry_run 一致性检查
    if (job_ir.arc.start != job_ir.arc.stop) {
        error_message = "ARC ON 与 ARC OFF 必须成对出现。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (job_ir.dry_run) {
        if (job_ir.arc.enabled) {
            error_message = "dry_run 模式下禁止启用真实引弧（arc.enabled 必须为 false）。";
            return ErrorCode::KA_JOB_INVALID;
        }
    } else {
        if (!job_ir.arc.enabled || !job_ir.arc.start || !job_ir.arc.stop) {
            error_message = "实焊任务必须启用电弧且包含成对的 ARC ON/OFF。";
            return ErrorCode::KA_JOB_INVALID;
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
        if (j.contains("max_lin_speed_m_s")) {
            config_.max_lin_speed_m_s = j.at("max_lin_speed_m_s").get<double>();
        }
        if (j.contains("max_weld_speed_m_s")) {
            config_.max_weld_speed_m_s = j.at("max_weld_speed_m_s").get<double>();
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
    } catch (const std::exception &e) {
        std::cerr << "[JobValidator] 解析配置文件失败，保留默认参数："
                  << e.what() << std::endl;
    }
}