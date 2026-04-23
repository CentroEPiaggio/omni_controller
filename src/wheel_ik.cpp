#include "omni_controller/wheel_ik.hpp"

#include <cmath>
#include <stdexcept>

namespace omni_controller {

// ─── MecanumIK ──────────────────────────────────────────────────────────────

void MecanumIK::configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names)
{
    if (wheel_names.size() != 4) {
        throw std::invalid_argument(
            "MecanumIK requires exactly 4 wheels, got " + std::to_string(wheel_names.size())
        );
    }

    double wr = config.wheel_rad;
    double ds_x = config.driveshaft_x;
    double ds_y = config.driveshaft_y;
    double ma = config.mecanum_angle * (M_PI / 180.0);

    double cot_ma = 1.0 / std::tan(ma);
    double rot_term = ds_x - ds_y * cot_ma;

    // Inverse kinematics: [vx, vy, omega] → wheel velocities
    // Wheel order: LF, LH, RF, RH
    base2wheel_[0][0] = -1.0 / wr; // LF
    base2wheel_[0][1] = 1.0 / wr;
    base2wheel_[0][2] = rot_term / wr;

    base2wheel_[1][0] = -1.0 / wr; // LH
    base2wheel_[1][1] = -1.0 / wr;
    base2wheel_[1][2] = rot_term / wr;

    base2wheel_[2][0] = 1.0 / wr; // RF
    base2wheel_[2][1] = 1.0 / wr;
    base2wheel_[2][2] = rot_term / wr;

    base2wheel_[3][0] = 1.0 / wr; // RH
    base2wheel_[3][1] = -1.0 / wr;
    base2wheel_[3][2] = rot_term / wr;

    // Forward kinematics: wheel velocities → [vx, vy, omega]
    //                      LF       LH       RF       RH
    odom_[0][0] = -wr / 4.0;
    odom_[0][1] = -wr / 4.0;
    odom_[0][2] = wr / 4.0;
    odom_[0][3] = wr / 4.0;

    odom_[1][0] = wr / 4.0;
    odom_[1][1] = -wr / 4.0;
    odom_[1][2] = wr / 4.0;
    odom_[1][3] = -wr / 4.0;

    odom_[2][0] = wr / (4.0 * rot_term);
    odom_[2][1] = wr / (4.0 * rot_term);
    odom_[2][2] = wr / (4.0 * rot_term);
    odom_[2][3] = wr / (4.0 * rot_term);
}

std::vector<double> MecanumIK::inverse(double vx, double vy, double omega) const
{
    std::vector<double> wheel_vels(4);
    double base_vel[3] = {vx, vy, omega};
    for (int i = 0; i < 4; i++) {
        double w_v = 0.0;
        for (int j = 0; j < 3; j++) {
            w_v += base2wheel_[i][j] * base_vel[j];
        }
        wheel_vels[i] = w_v;
    }
    return wheel_vels;
}

std::array<double, 3> MecanumIK::forward(const std::vector<double>& wheel_velocities) const
{
    std::array<double, 3> odom_vel = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            odom_vel[i] += odom_[i][j] * wheel_velocities[j];
        }
    }
    return odom_vel;
}

// ─── DifferentialIK ─────────────────────────────────────────────────────────

void DifferentialIK::configure(
    const WheelIKConfig& config, const std::vector<std::string>& wheel_names
)
{
    if (wheel_names.size() != 2) {
        throw std::invalid_argument(
            "DifferentialIK requires exactly 2 wheels, got " + std::to_string(wheel_names.size())
        );
    }
    wheel_rad_ = config.wheel_rad;
    track_width_ = config.track_width;
}

std::vector<double> DifferentialIK::inverse(double vx, double /*vy*/, double omega) const
{
    // wheel_names order: [left, right]
    double v_left = (vx - omega * track_width_ / 2.0) / wheel_rad_;
    double v_right = (vx + omega * track_width_ / 2.0) / wheel_rad_;
    return {v_left, v_right};
}

std::array<double, 3> DifferentialIK::forward(const std::vector<double>& wheel_velocities) const
{
    double v_left = wheel_velocities[0] * wheel_rad_;
    double v_right = wheel_velocities[1] * wheel_rad_;
    double vx = (v_left + v_right) / 2.0;
    double omega = (v_right - v_left) / track_width_;
    return {vx, 0.0, omega};
}

// ─── Factory ────────────────────────────────────────────────────────────────

std::unique_ptr<WheelIK> create_wheel_ik(const std::string& type)
{
    if (type == "mecanum") {
        return std::make_unique<MecanumIK>();
    } else if (type == "differential") {
        return std::make_unique<DifferentialIK>();
    } else if (type == "none") {
        return nullptr;
    }
    throw std::invalid_argument(
        "Unknown wheel IK type: '" + type + "'. Use 'mecanum', 'differential', or 'none'."
    );
}

} // namespace omni_controller
