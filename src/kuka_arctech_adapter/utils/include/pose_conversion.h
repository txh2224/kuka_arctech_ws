#ifndef POSE_CONVERSION_H
#define POSE_CONVERSION_H

#include <array>

#include "job_ir.h"

namespace utils {

struct RosPose {
    std::array<double, 3> position_m;
    std::array<double, 4> quaternion_xyzw;
};

// 返回 false 时四元数模长非法，out_pose 保持不变。
bool ros_to_kuka_pose(const RosPose &pose, KukaPose &out_pose);
RosPose kuka_to_ros_pose(const KukaPose &pose);

}  // namespace utils

#endif  // POSE_CONVERSION_H
