#ifndef JOB_IR_H
#define JOB_IR_H

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

//这个头文件主要是定义了IR结构体和相关的枚举类型，方便后续的检查任务是否合法的函数使用

struct KukaPose {
    double x{std::numeric_limits<double>::quiet_NaN()};
    double y{std::numeric_limits<double>::quiet_NaN()};
    double z{std::numeric_limits<double>::quiet_NaN()};
    double a{std::numeric_limits<double>::quiet_NaN()};
    double b{std::numeric_limits<double>::quiet_NaN()};
    double c{std::numeric_limits<double>::quiet_NaN()};
};

enum class MotionType {
    UNKNOWN,
    PTP,
    LIN
};

enum class WeldPathType {
    UNKNOWN,
    LIN_POLYLINE
};

struct ApproachMotion {
    MotionType type{MotionType::UNKNOWN}; 
    KukaPose pose;
    double speed{-1.0};
};

struct WeldPath {
    WeldPathType type{WeldPathType::UNKNOWN}; 
    std::vector<KukaPose> points;            
    double speed{-1.0};
};

struct RetractMotion {
    MotionType type{MotionType::UNKNOWN}; 
    KukaPose pose;
    double speed{-1.0};
};

struct ArcConfig {
    bool enabled{false}; 
    bool start{false};  
    bool stop{false};    
};


struct KukaArcTechJobIR {
    //这个是由任务书7章定义的IR结构体
    std::string job_id;
    std::string trace_id;
    std::string program_name;//这个是.src和.dat前的那个名字
    bool dry_run{true};

    int tool_id{-1};//焊枪编号？
    int base_id{-1};//工作台编号？
    uint32_t seam_id{0};//焊缝编号？
    std::string weld_data_ref; //焊接数据？

    //三段运动轨迹
    ApproachMotion approach;
    WeldPath weld_path;
    RetractMotion retract;
    ArcConfig arc;
};

#endif // JOB_IR_H
