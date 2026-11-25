#ifndef JOB_IR_H
#define JOB_IR_H

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <variant>
#include <vector>

//这个头文件主要是定义了IR结构体和相关的枚举类型，方便后续的检查任务是否合法的函数使用
//太cs了，B任务书给的结构体定义根本用不了A部分发的原子指令

// KUKA 笛卡尔位姿（内部统一约定：位置 mm，欧拉角 A/B/C 为 deg）
struct KukaPose {
    double x{std::numeric_limits<double>::quiet_NaN()};
    double y{std::numeric_limits<double>::quiet_NaN()};
    double z{std::numeric_limits<double>::quiet_NaN()};
    double a{std::numeric_limits<double>::quiet_NaN()};
    double b{std::numeric_limits<double>::quiet_NaN()};
    double c{std::numeric_limits<double>::quiet_NaN()};
};

//instruction_id为 ROB-010 代表关节运动，需要关节名和目标角度，速度和加速度
struct MoveJointStep {
    std::array<std::string, 6> joint_names;
    std::array<double, 6> target_rad{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
    double max_velocity_rad_s{-1.0};
    double max_acceleration_rad_s2{-1.0};
};

//instruction_id为 ROB-011 代表直线运动，需要坐标系、工具、目标位姿、速度和加速度
struct MoveLinearStep {
    KukaPose target_pose;
    double speed_m_s{-1.0};
    double acceleration_m_s2{-1.0};
};

// instruction_id为 ROB-015 代表跟随路径，需要坐标系、工具和一系列路径点、速度
struct FollowPathStep {
    std::vector<KukaPose> waypoints;
    double speed_m_s{-1.0};
};

// instruction_id为 WLD-003 代表选择焊接配方
struct RecipeSelectStep {
    std::string program_id;     // A 组下发的配方程序 ID（如 "WELD_PROGRAM_01"）
    std::string weld_data_ref;  // 映射后的 KUKA WDAT 名（如 "WDAT_DEMO_01"）
};

// instruction_id为 WLD-015 代表起弧，需要起弧配置文件ID和租约时间
struct ArcStartStep {
    std::string start_profile_id;
    uint32_t lease_ms{0};
};

// instruction_id为 WLD-016 代表收弧，需要收弧配置文件ID
struct ArcStopStep {
    std::string crater_profile_id;
};

// instruction_id为 WLD-024 代表测量过程量，需要测量通道名列表
struct MeasureProcessStep {
    std::vector<std::string> channels; // 如 ["current", "voltage"]
};

//这里用std::variant来定义一个联合类型，表示不同类型的步骤，比使用继承更方便，避免了虚函数的开销
using StepPayload = std::variant<
    MoveJointStep,
    MoveLinearStep,
    FollowPathStep,
    RecipeSelectStep,
    ArcStartStep,
    ArcStopStep,
    MeasureProcessStep>;

// 单条原子指令解析后的强类型步骤（保留 index / instance_id 以便按序校验和回填 step_results）
struct JobStep {
    uint32_t index{0};
    std::string instruction_id; // 如 "ROB-010"
    std::string instance_id;    // 如 "inst_mock_01"
    std::string capability_id;  // 如 "robot.motion.move_joint"
    uint32_t timeout_ms{0};     // 单步执行超时（0 表示不限）
    uint32_t settle_duration_ms{200}; // 来自 constraint_json.settle_duration_ms（默认 200ms）
    std::string constraint_json;
    std::string safety_json;
    std::string result_contract_json;
    StepPayload payload;
};

struct KukaArcTechJobIR {
    //这个是由任务书7章与AB通信定义共同确定的IR结构体
    std::string job_id;
    std::string trace_id;
    std::string program_name;//这个是.src和.dat前的那个名字
    bool dry_run{true};

    int tool_id{-1};//焊枪编号？
    int base_id{-1};//工作台编号？
    uint32_t seam_id{0};//焊缝编号？

    // 按 Goal.instructions 原序保存的强类型步骤列表
    std::vector<JobStep> steps;
    //扎啤吧，为什么一个action里面可以放一个可以放多个不早点说。
};

// 判断作业是否全部由只读过程测量指令（WLD-024）组成：应走只读查询路径，不生成/不部署运动程序
inline bool is_read_only_query_job(const KukaArcTechJobIR &job_ir) {
    if (job_ir.steps.empty()) {
        return false;
    }
    for (const auto &step : job_ir.steps) {
        if (!std::holds_alternative<MeasureProcessStep>(step.payload)) {
            return false;
        }
    }
    return true;
}

#endif // JOB_IR_H
