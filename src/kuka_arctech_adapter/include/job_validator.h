#ifndef JOB_VALIDATOR_H
#define JOB_VALIDATOR_H

//我其实不是很理解为什么任务书要求把这个检查任务是否合法的函数单独分一个文件
//可能以后检查任务是否合法的逻辑会变得很复杂吧
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>

#include <atomic_adapter_interfaces/action/execute_backend_job.hpp>
#include "error_code.h"
#include "job_ir.h"

//目前不知道数据的合法范围时多少
struct vali_config{

    std::size_t max_program_name_length{24};//程序名的最大长度
    int max_tool_id{16};//焊枪编号的最大值
    int max_base_id{32};//坐标系编号的最大值
    uint32_t max_seam_id{65535};//ArcTech 3.8 PLC 焊缝许可编号上限（0=未启用，1~65535=启用）
    std::size_t max_weld_path_points{100};//焊接路径点的最大数量

    double max_ptp_speed_percent{100.0};//点位移动速度的最大百分比
    double max_joint_velocity_rad_s{3.14};//关节运动最大角速度（rad/s）
    double max_joint_acceleration_rad_s2{10.0};//关节运动最大角加速度（rad/s^2）
    double max_lin_speed_m_s{2.0};//线性移动速度的最大值（m/s）
    double max_lin_acceleration_m_s2{5.0};//线性移动加速度的最大值（m/s^2）
    double max_weld_speed_m_s{0.2};//焊接速度的最大值（m/s）

    double min_joint_rad{-6.283185307179586};
    double max_joint_rad{6.283185307179586};
    double min_x_mm{-3000.0};
    double max_x_mm{3000.0};
    double min_y_mm{-3000.0};
    double max_y_mm{3000.0};
    double min_z_mm{-1000.0};
    double max_z_mm{3000.0};
    double min_angle_deg{-180.0};
    double max_angle_deg{180.0};

    // 非空时强制要求 weld_data_ref 必须在白名单内（KA-T05）
    std::unordered_set<std::string> wdat_allowlist;
    // 允许读取的过程量通道名（WLD-024）
    std::unordered_set<std::string> allowed_measure_channels{"current", "voltage"};
};

class JobValidator {
public:
    JobValidator() {//构造时直接读取配置文件
        read_config();
    }
    ~JobValidator() = default;

    //检查转成 IR 前所需的 Goal 基本字段
    ErrorCode validate(
        const atomic_adapter_interfaces::action::ExecuteBackendJob::Goal &goal,
        std::string &error_message) const;
    //检查 IR 内部字段是否合法
    ErrorCode validate_ir(
        const struct KukaArcTechJobIR &job_ir,
        std::string &error_message) const;

private:
    //检查程序名是否合法
    ErrorCode validate_program_name(
        const std::string &program_name,
        std::string &error_message) const;
    //检查位姿是否合法
    ErrorCode validate_pose(
        const KukaPose &pose,
        const std::string &label,
        std::string &error_message) const;

    //各原子步骤的参数校验
    ErrorCode validate_step(const MoveJointStep &step, std::string &error_message) const;
    ErrorCode validate_step(const MoveLinearStep &step, std::string &error_message) const;
    ErrorCode validate_step(const FollowPathStep &step, std::string &error_message) const;
    ErrorCode validate_step(const RecipeSelectStep &step, std::string &error_message) const;
    ErrorCode validate_step(const ArcStartStep &step, std::string &error_message) const;
    ErrorCode validate_step(const ArcStopStep &step, std::string &error_message) const;
    ErrorCode validate_step(const MeasureProcessStep &step, std::string &error_message) const;

    //多步序列的时序与工艺约束校验
    ErrorCode validate_sequence(
        const struct KukaArcTechJobIR &job_ir,
        std::string &error_message) const;

    //读取配置参数
    void read_config();

    vali_config config_;
};

#endif // JOB_VALIDATOR_H
