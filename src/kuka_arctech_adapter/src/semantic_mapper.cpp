#include "semantic_mapper.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
//这一部分也是哈基米写的

// 将 A 组的 {position_m: [x,y,z], quaternion_xyzw: [x,y,z,w]} 转换为 KUKA 位姿（mm + ZYX 欧拉角 deg）
bool parse_pose_json(
    const nlohmann::json &pose_obj,
    KukaPose &out_pose,
    std::string &error_message) {
    if (!pose_obj.is_object()) {
        error_message = "位姿字段必须为 JSON 对象。";
        return false;
    }
    if (!pose_obj.contains("position_m") || !pose_obj.at("position_m").is_array() ||
        pose_obj.at("position_m").size() != 3) {
        error_message = "位姿缺少合法的 position_m（长度为 3 的数组）。";
        return false;
    }
    if (!pose_obj.contains("quaternion_xyzw") || !pose_obj.at("quaternion_xyzw").is_array() ||
        pose_obj.at("quaternion_xyzw").size() != 4) {
        error_message = "位姿缺少合法的 quaternion_xyzw（长度为 4 的数组）。";
        return false;
    }

    const auto &pos = pose_obj.at("position_m");
    const auto &quat = pose_obj.at("quaternion_xyzw");
    for (std::size_t i = 0; i < 3; ++i) {
        if (!pos[i].is_number()) {
            error_message = "position_m 元素必须为数值。";
            return false;
        }
    }
    for (std::size_t i = 0; i < 4; ++i) {
        if (!quat[i].is_number()) {
            error_message = "quaternion_xyzw 元素必须为数值。";
            return false;
        }
    }

    const double qx = quat[0].get<double>();
    const double qy = quat[1].get<double>();
    const double qz = quat[2].get<double>();
    const double qw = quat[3].get<double>();
    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!std::isfinite(norm) || norm < 1e-9) {
        error_message = "quaternion_xyzw 模长非法（不能为零向量或 NaN）。";
        return false;
    }

    const double x = qx / norm;
    const double y = qy / norm;
    const double z = qz / norm;
    const double w = qw / norm;

    // 位置：m -> mm
    out_pose.x = pos[0].get<double>() * 1000.0;
    out_pose.y = pos[1].get<double>() * 1000.0;
    out_pose.z = pos[2].get<double>() * 1000.0;

    // 姿态：四元数 -> KUKA 内旋 ZYX 欧拉角（A=Yaw(Z), B=Pitch(Y), C=Roll(X)），单位 deg
    const double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    const double pitch_rad = std::asin(sinp);

    double yaw_rad = 0.0;
    double roll_rad = 0.0;
    if (std::abs(sinp) >= 1.0 - 1e-7) {
        // 万向节锁（B = ±90°）：固定 C = 0，直接从旋转矩阵元素求解 A，避免姿态失真
        const double r01 = 2.0 * (x * y - w * z);
        const double r11 = 1.0 - 2.0 * (x * x + z * z);
        yaw_rad = std::atan2(-r01, r11);
        roll_rad = 0.0;
    } else {
        const double sinr_cosp = 2.0 * (w * x + y * z);
        const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        roll_rad = std::atan2(sinr_cosp, cosr_cosp);

        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    }

    out_pose.a = yaw_rad * kRadToDeg;
    out_pose.b = pitch_rad * kRadToDeg;
    out_pose.c = roll_rad * kRadToDeg;
    return true;
}

bool extract_required_string(
    const nlohmann::json &obj,
    const std::string &key,
    const std::string &field_label,
    std::string &out,
    std::string &error_message) {
    if (!obj.contains(key)) {
        error_message = "缺少 " + field_label + "。";
        return false;
    }
    if (!obj.at(key).is_string()) {
        error_message = field_label + " 类型错误（必须为字符串）。";
        return false;
    }
    out = obj.at(key).get<std::string>();
    if (out.empty()) {
        error_message = field_label + " 不能为空。";
        return false;
    }
    return true;
}

bool parse_json_object(
    const std::string &raw,
    const std::string &field_name,
    nlohmann::json &out,
    std::string &error_message) {
    if (raw.empty()) {
        out = nlohmann::json::object();
        return true;
    }
    try {
        out = nlohmann::json::parse(raw);
    } catch (const std::exception &e) {
        error_message = field_name + " JSON 解析失败：" + e.what();
        return false;
    }
    if (!out.is_object()) {
        error_message = field_name + " 必须为 JSON 对象。";
        return false;
    }
    return true;
}

}  // namespace

void SemanticMapper::read_config() {
    std::filesystem::path config_path;
    try {
        const auto share_dir =
            ament_index_cpp::get_package_share_directory("kuka_arctech_adapter");
        config_path = std::filesystem::path(share_dir) / "config" / "mapper.json";
    } catch (const std::exception &) {
        config_path = std::filesystem::path("config") / "mapper.json";
    }

    if (!std::filesystem::exists(config_path)) {
        std::cerr << "[SemanticMapper] 未找到配置文件，使用默认映射："
                  << config_path << std::endl;
        return;
    }

    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        std::cerr << "[SemanticMapper] 无法打开配置文件，使用默认映射："
                  << config_path << std::endl;
        return;
    }

    try {
        nlohmann::json j;
        ifs >> j;

        if (j.contains("program_name_prefix") && j.at("program_name_prefix").is_string()) {
            config_.program_name_prefix = j.at("program_name_prefix").get<std::string>();
        }
        if (j.contains("max_program_name_length") &&
            j.at("max_program_name_length").is_number_unsigned()) {
            config_.max_program_name_length =
                j.at("max_program_name_length").get<std::size_t>();
        }
        if (j.contains("default_seam_id") && j.at("default_seam_id").is_number_unsigned()) {
            config_.default_seam_id = j.at("default_seam_id").get<uint32_t>();
        }
        if (j.contains("tool_id_map") && j.at("tool_id_map").is_object()) {
            config_.tool_id_map.clear();
            for (const auto &[k, v] : j.at("tool_id_map").items()) {
                config_.tool_id_map[k] = v.get<int>();
            }
        }
        if (j.contains("frame_id_map") && j.at("frame_id_map").is_object()) {
            config_.frame_id_map.clear();
            for (const auto &[k, v] : j.at("frame_id_map").items()) {
                config_.frame_id_map[k] = v.get<int>();
            }
        }
        if (j.contains("recipe_wdat_map") && j.at("recipe_wdat_map").is_object()) {
            config_.recipe_wdat_map.clear();
            for (const auto &[k, v] : j.at("recipe_wdat_map").items()) {
                config_.recipe_wdat_map[k] = v.get<std::string>();
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "[SemanticMapper] 解析配置文件失败，保留默认映射："
                  << e.what() << std::endl;
    }
}

std::string SemanticMapper::make_program_name(const std::string &job_id) const {
    std::string sanitized;
    sanitized.reserve(config_.program_name_prefix.size() + job_id.size());

    auto append_sanitized = [&](const std::string &src) {
        for (char ch : src) {
            const unsigned char uch = static_cast<unsigned char>(ch);
            if (std::isalnum(uch) || ch == '_') {
                sanitized.push_back(static_cast<char>(std::toupper(uch)));
            } else {
                sanitized.push_back('_');
            }
        }
    };

    append_sanitized(config_.program_name_prefix);
    append_sanitized(job_id);

    if (sanitized.empty() || !std::isalpha(static_cast<unsigned char>(sanitized.front()))) {
        sanitized = "P_" + sanitized;
    }
    if (sanitized.size() > config_.max_program_name_length) {
        sanitized.resize(config_.max_program_name_length);
    }
    return sanitized;
}

ErrorCode SemanticMapper::bind_tool_and_base(
    const std::string &tool_name,
    const std::string &frame_name,
    KukaArcTechJobIR &ir,
    std::string &error_message) const {
    if (tool_name.empty()) {
        error_message = "笛卡尔运动缺少 object_json.tool_id。";
        return ErrorCode::KA_JOB_INVALID;
    }
    if (frame_name.empty()) {
        error_message = "笛卡尔运动缺少 object_json.frame_id。";
        return ErrorCode::KA_JOB_INVALID;
    }

    const auto tool_it = config_.tool_id_map.find(tool_name);
    if (tool_it == config_.tool_id_map.end()) {
        error_message = "未知的 tool_id 映射：" + tool_name;
        return ErrorCode::KA_JOB_INVALID;
    }
    const auto frame_it = config_.frame_id_map.find(frame_name);
    if (frame_it == config_.frame_id_map.end()) {
        error_message = "未知的 frame_id 映射：" + frame_name;
        return ErrorCode::KA_JOB_INVALID;
    }

    const int mapped_tool = tool_it->second;
    const int mapped_base = frame_it->second;

    if (ir.tool_id < 0) {
        ir.tool_id = mapped_tool;
    } else if (ir.tool_id != mapped_tool) {
        error_message = "同一作业内不允许切换 tool_id：" + tool_name;
        return ErrorCode::KA_JOB_INVALID;
    }

    if (ir.base_id < 0) {
        ir.base_id = mapped_base;
    } else if (ir.base_id != mapped_base) {
        error_message = "同一作业内不允许切换 frame_id：" + frame_name;
        return ErrorCode::KA_JOB_INVALID;
    }

    return ErrorCode::OK;
}

ErrorCode SemanticMapper::map(
    const atomic_adapter_interfaces::action::ExecuteBackendJob::Goal &goal,
    KukaArcTechJobIR &out_ir,
    std::string &error_message) const {
    error_message.clear();

    KukaArcTechJobIR ir;
    ir.job_id = goal.job_id;
    ir.trace_id = goal.trace_id;
    ir.program_name = make_program_name(goal.job_id);
    ir.dry_run = goal.dry_run;
    ir.steps.reserve(goal.instructions.size());

    for (std::size_t i = 0; i < goal.instructions.size(); ++i) {
        const auto &inst = goal.instructions[i];
        const std::string step_prefix = "第 " + std::to_string(i) + " 步 (" +
                                        inst.instruction_id + "/" + inst.capability_id + ")：";

        nlohmann::json obj_json;
        nlohmann::json param_json;
        if (!parse_json_object(inst.object_json, "object_json", obj_json, error_message) ||
            !parse_json_object(inst.parameter_json, "parameter_json", param_json, error_message)) {
            error_message = step_prefix + error_message;
            return ErrorCode::KA_JOB_INVALID;
        }

        JobStep step;
        step.index = static_cast<uint32_t>(i);
        step.instruction_id = inst.instruction_id;
        step.instance_id = inst.instance_id;
        step.capability_id = inst.capability_id;

        if (inst.capability_id == "robot.motion.move_joint") {
            MoveJointStep payload;
            if (!param_json.contains("joint_names") ||
                !param_json.at("joint_names").is_array() ||
                param_json.at("joint_names").size() != 6) {
                error_message = step_prefix + "缺少合法的 joint_names（长度须为 6）。";
                return ErrorCode::KA_JOB_INVALID;
            }
            if (!param_json.contains("target_rad") ||
                !param_json.at("target_rad").is_array() ||
                param_json.at("target_rad").size() != 6) {
                error_message = step_prefix + "缺少合法的 target_rad（长度须为 6）。";
                return ErrorCode::KA_JOB_INVALID;
            }
            for (std::size_t j = 0; j < 6; ++j) {
                if (!param_json.at("joint_names")[j].is_string() ||
                    !param_json.at("target_rad")[j].is_number()) {
                    error_message = step_prefix + "joint_names 或 target_rad 元素类型非法。";
                    return ErrorCode::KA_JOB_INVALID;
                }
                payload.joint_names[j] = param_json.at("joint_names")[j].get<std::string>();
                payload.target_rad[j] = param_json.at("target_rad")[j].get<double>();
            }
            if (param_json.contains("max_velocity_rad_s") &&
                param_json.at("max_velocity_rad_s").is_number()) {
                payload.max_velocity_rad_s = param_json.at("max_velocity_rad_s").get<double>();
            }
            if (param_json.contains("max_acceleration_rad_s2") &&
                param_json.at("max_acceleration_rad_s2").is_number()) {
                payload.max_acceleration_rad_s2 =
                    param_json.at("max_acceleration_rad_s2").get<double>();
            }
            step.payload = std::move(payload);

        } else if (inst.capability_id == "robot.motion.move_linear") {
            std::string tool_name;
            std::string frame_name;
            if (!extract_required_string(obj_json, "tool_id", "object_json.tool_id", tool_name, error_message) ||
                !extract_required_string(obj_json, "frame_id", "object_json.frame_id", frame_name, error_message)) {
                error_message = step_prefix + error_message;
                return ErrorCode::KA_JOB_INVALID;
            }
            if (const auto err = bind_tool_and_base(tool_name, frame_name, ir, error_message);
                err != ErrorCode::OK) {
                error_message = step_prefix + error_message;
                return err;
            }

            MoveLinearStep payload;
            if (!param_json.contains("target_pose") ||
                !parse_pose_json(param_json.at("target_pose"), payload.target_pose, error_message)) {
                error_message = step_prefix + (error_message.empty() ? "缺少 target_pose。" : error_message);
                return ErrorCode::KA_JOB_INVALID;
            }
            if (param_json.contains("speed_m_s") && param_json.at("speed_m_s").is_number()) {
                payload.speed_m_s = param_json.at("speed_m_s").get<double>();
            }
            if (param_json.contains("acceleration_m_s2") &&
                param_json.at("acceleration_m_s2").is_number()) {
                payload.acceleration_m_s2 = param_json.at("acceleration_m_s2").get<double>();
            }
            step.payload = std::move(payload);

        } else if (inst.capability_id == "robot.motion.follow_path") {
            std::string tool_name;
            std::string frame_name;
            if (!extract_required_string(obj_json, "tool_id", "object_json.tool_id", tool_name, error_message) ||
                !extract_required_string(obj_json, "frame_id", "object_json.frame_id", frame_name, error_message)) {
                error_message = step_prefix + error_message;
                return ErrorCode::KA_JOB_INVALID;
            }
            if (const auto err = bind_tool_and_base(tool_name, frame_name, ir, error_message);
                err != ErrorCode::OK) {
                error_message = step_prefix + error_message;
                return err;
            }

            FollowPathStep payload;
            if (!param_json.contains("waypoints") || !param_json.at("waypoints").is_array()) {
                error_message = step_prefix + "缺少 waypoints 数组。";
                return ErrorCode::KA_JOB_INVALID;
            }
            payload.waypoints.reserve(param_json.at("waypoints").size());
            for (const auto &wp : param_json.at("waypoints")) {
                KukaPose pose;
                if (!parse_pose_json(wp, pose, error_message)) {
                    error_message = step_prefix + error_message;
                    return ErrorCode::KA_JOB_INVALID;
                }
                payload.waypoints.push_back(pose);
            }
            if (param_json.contains("speed_m_s") && param_json.at("speed_m_s").is_number()) {
                payload.speed_m_s = param_json.at("speed_m_s").get<double>();
            }
            step.payload = std::move(payload);

        } else if (inst.capability_id == "welder.recipe.select") {
            RecipeSelectStep payload;
            if (!extract_required_string(
                    param_json, "program_id", "parameter_json.program_id",
                    payload.program_id, error_message)) {
                error_message = step_prefix + error_message;
                return ErrorCode::KA_JOB_INVALID;
            }
            const auto it = config_.recipe_wdat_map.find(payload.program_id);
            if (it == config_.recipe_wdat_map.end() || it->second.empty()) {
                error_message = step_prefix + "未在 mapper.json 中注册对应的 WDAT 配方映射：" +
                                payload.program_id;
                return ErrorCode::KA_JOB_INVALID;
            }
            payload.weld_data_ref = it->second;
            step.payload = std::move(payload);

        } else if (inst.capability_id == "welder.arc.start") {
            ArcStartStep payload;
            if (param_json.contains("start_profile_id") &&
                param_json.at("start_profile_id").is_string()) {
                payload.start_profile_id = param_json.at("start_profile_id").get<std::string>();
            }
            if (param_json.contains("lease_ms") && param_json.at("lease_ms").is_number_unsigned()) {
                payload.lease_ms = param_json.at("lease_ms").get<uint32_t>();
            }
            if (ir.seam_id == 0) {
                ir.seam_id = config_.default_seam_id;
            }
            step.payload = std::move(payload);

        } else if (inst.capability_id == "welder.arc.stop") {
            ArcStopStep payload;
            if (param_json.contains("crater_profile_id") &&
                param_json.at("crater_profile_id").is_string()) {
                payload.crater_profile_id = param_json.at("crater_profile_id").get<std::string>();
            }
            step.payload = std::move(payload);

        } else if (inst.capability_id == "welder.measure.get_process") {
            MeasureProcessStep payload;
            if (param_json.contains("channels") && param_json.at("channels").is_array()) {
                for (const auto &ch : param_json.at("channels")) {
                    if (!ch.is_string()) {
                        error_message = step_prefix + "channels 元素必须为字符串。";
                        return ErrorCode::KA_JOB_INVALID;
                    }
                    payload.channels.push_back(ch.get<std::string>());
                }
            }
            step.payload = std::move(payload);

        } else {
            error_message = step_prefix + "不支持的 capability_id：" + inst.capability_id;
            return ErrorCode::KA_JOB_INVALID;
        }

        ir.steps.push_back(std::move(step));
    }

    out_ir = std::move(ir);
    return ErrorCode::OK;
}
