#ifndef OMNI_CONTROLLER_WHEEL_IK_HPP
#define OMNI_CONTROLLER_WHEEL_IK_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace omni_controller {

struct WheelIKConfig {
    double wheel_rad = 0.05;
    double driveshaft_x = 0.235;
    double driveshaft_y = 0.188;
    double mecanum_angle = 135.0; // degrees
    double track_width = 0.4;
};

class WheelIK {
public:
    virtual ~WheelIK() = default;

    virtual void
    configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names) = 0;

    /// Compute wheel velocities from base twist (vx, vy, omega)
    virtual std::vector<double> inverse(double vx, double vy, double omega) const = 0;

    /// Compute base twist (vx, vy, omega) from wheel velocities
    virtual std::array<double, 3> forward(const std::vector<double>& wheel_velocities) const = 0;

    virtual size_t num_wheels() const = 0;
};

/// 4-wheel mecanum IK (ported from omni_vel_controller)
class MecanumIK: public WheelIK {
public:
    void
    configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names) override;

    std::vector<double> inverse(double vx, double vy, double omega) const override;
    std::array<double, 3> forward(const std::vector<double>& wheel_velocities) const override;
    size_t num_wheels() const override { return 4; }

private:
    double base2wheel_[4][3] = {};
    double odom_[3][4] = {};
};

/// 2-wheel differential IK
class DifferentialIK: public WheelIK {
public:
    void
    configure(const WheelIKConfig& config, const std::vector<std::string>& wheel_names) override;

    std::vector<double> inverse(double vx, double vy, double omega) const override;
    std::array<double, 3> forward(const std::vector<double>& wheel_velocities) const override;
    size_t num_wheels() const override { return 2; }

private:
    double wheel_rad_ = 0.05;
    double track_width_ = 0.4;
};

/// Factory: create WheelIK by name ("mecanum", "differential")
/// Returns nullptr for "none"
std::unique_ptr<WheelIK> create_wheel_ik(const std::string& type);

} // namespace omni_controller

#endif // OMNI_CONTROLLER_WHEEL_IK_HPP
