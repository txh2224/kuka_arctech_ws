#include "pose_conversion.h"

#include <algorithm>
#include <cmath>

namespace utils {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMillimetersPerMeter = 1000.0;

}  // namespace

bool ros_to_kuka_pose(const RosPose &pose, KukaPose &out_pose) {
    const auto &[qx, qy, qz, qw] = pose.quaternion_xyzw;
    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!std::isfinite(norm) || norm < 1e-9) {
        return false;
    }

    const double x = qx / norm;
    const double y = qy / norm;
    const double z = qz / norm;
    const double w = qw / norm;

    out_pose.x = pose.position_m[0] * kMillimetersPerMeter;
    out_pose.y = pose.position_m[1] * kMillimetersPerMeter;
    out_pose.z = pose.position_m[2] * kMillimetersPerMeter;

    // KUKA 使用内旋 ZYX 欧拉角，A、B、C 分别绕 Z、Y、X 轴旋转。
    const double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    const double pitch_rad = std::asin(sinp);

    double yaw_rad = 0.0;
    double roll_rad = 0.0;
    if (std::abs(sinp) >= 1.0 - 1e-7) {
        // 万向节锁时固定 C 为 0，从旋转矩阵元素求解 A。
        const double r01 = 2.0 * (x * y - w * z);
        const double r11 = 1.0 - 2.0 * (x * x + z * z);
        yaw_rad = std::atan2(-r01, r11);
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

RosPose kuka_to_ros_pose(const KukaPose &pose) {
    const double yaw = pose.a * kDegToRad * 0.5;
    const double pitch = pose.b * kDegToRad * 0.5;
    const double roll = pose.c * kDegToRad * 0.5;

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);

    const double qw = cr * cp * cy + sr * sp * sy;
    const double qx = sr * cp * cy - cr * sp * sy;
    const double qy = cr * sp * cy + sr * cp * sy;
    const double qz = cr * cp * sy - sr * sp * cy;

    return {
        {pose.x / kMillimetersPerMeter,
         pose.y / kMillimetersPerMeter,
         pose.z / kMillimetersPerMeter},
        {qx, qy, qz, qw},
    };
}

}  // namespace utils
