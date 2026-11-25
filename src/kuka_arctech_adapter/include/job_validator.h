#ifndef JOB_VALIDATOR_H
#define JOB_VALIDATOR_H

//我其实不是很理解为什么任务书要求把这个检查任务是否合法的函数单独分一个文件
//可能以后检查任务是否合法的逻辑会变得很复杂吧
#include <cstddef>
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
    std::size_t max_weld_path_points{100};//焊接路径点的最大数量

    double max_ptp_speed_percent{100.0};//点位移动速度的最大百分比
    double max_lin_speed_m_s{2.0};//线性移动速度的最大值（m/s）
    double max_weld_speed_m_s{0.2};//焊接速度的最大值（m/s）

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
    //读取配置参数
    void read_config();

    vali_config config_;
};

#endif // JOB_VALIDATOR_H
