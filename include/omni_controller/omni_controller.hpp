#ifndef OMNI_CONTROLLER_HPP
#define OMNI_CONTROLLER_HPP

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pi3hat_moteus_int_msgs/msg/distributors_state.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"
#include "pi3hat_moteus_int_msgs/msg/packet_pass.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "omni_controller/wheel_ik.hpp"

#include <chrono>

namespace omni_controller {

// Local copies of custom HW interface names (avoids depending on pi3hat_hw_interface)
namespace hw_if {
constexpr char KP_SCALE[] = "kp_scale_value";
constexpr char KD_SCALE[] = "kd_scale_value";
constexpr char TEMPERATURE[] = "temperature";
constexpr char Q_CURRENT[] = "q_current";
constexpr char VOLTAGE[] = "voltage";
constexpr char CURRENT[] = "current";
constexpr char VALIDITY_LOSS[] = "validity_loss";
constexpr char PACKAGE_LOSS[] = "package_loss";
constexpr char CYCLE_DUR[] = "cycle_duration";
} // namespace hw_if

enum ControllerState {
    INACTIVE = 0,
    TRANSITION,
    ACTIVE,
};

enum SafetyState {
    SAFETY_NORMAL = 0,
    SAFETY_WARNING,
    SAFETY_CRITICAL,
    SAFETY_DAMPING,
    SAFETY_STOPPED,
};

struct JointTargets {
    double q_rest = 0.0;
    double q_stand = 0.0;
};

enum TransitionTarget { TARGET_REST = 0, TARGET_STAND };

enum WheelMode { WHEEL_IK = 0, WHEEL_DIRECT };

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using TransactionService = std_srvs::srv::SetBool;
using JointsCommand = pi3hat_moteus_int_msgs::msg::JointsCommand;
using JointsStates = pi3hat_moteus_int_msgs::msg::JointsStates;
using PacketPass = pi3hat_moteus_int_msgs::msg::PacketPass;
using DistributorsState = pi3hat_moteus_int_msgs::msg::DistributorsState;

class OmniController: public controller_interface::ControllerInterface {
public:
    OmniController() = default;
    ~OmniController() override = default;

    CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type
    update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // ─── Configuration ──────────────────────────────────────────────────
    std::vector<std::string> wheel_joints_;
    std::vector<std::vector<std::string>> wheel_groups_; // IK index → joint names
    std::vector<std::string> joints_;
    std::vector<std::string> all_motor_joints_; // wheel_joints_ + joints_
    std::vector<std::string> distributor_names_;
    std::vector<std::string> second_encoder_joints_;
    std::vector<bool> se_flag_; // per motor joint: has second encoder?

    bool has_wheels_ = false;
    bool has_legs_ = false;
    bool has_distributors_ = false;
    bool has_transitions_ = false;
    bool transition_completed_ = false;
    bool sim_flag_ = false;
    double sim_kp_ = 50.0;
    double sim_kd_ = 1.0;
    bool pub_odom_ = false;
    bool pub_performance_ = true;

    // ─── Safety parameters ──────────────────────────────────────────────
    bool safety_enabled_ = true;
    double temp_warning_threshold_ = 50.0;
    double temp_critical_threshold_ = 57.0;
    double battery_min_voltage_ = 24.0;
    double temp_recovery_hysteresis_ = 5.0;
    double volt_recovery_hysteresis_ = 1.0;
    int ema_window_samples_ = 500;
    double ema_alpha_ = 0.0;
    std::string critical_strategy_ = "damping";
    double damping_duration_ = 3.0;
    double joints_reference_timeout_ = 0.5;
    double heartbeat_timeout_ = 1.0;

    // ─── Wheel IK / direct mode ────────────────────────────────────────
    std::unique_ptr<WheelIK> wheel_ik_;
    WheelMode wheel_mode_ = WHEEL_IK;

    // ─── State machine ──────────────────────────────────────────────────
    ControllerState c_stt_ = ControllerState::INACTIVE;
    std::atomic<int> dl_miss_count_{0};

    // ─── Transitions (rest / stand) ────────────────────────────────────
    double rest_duration_ = 5.0;
    double stand_duration_ = 5.0;
    std::map<std::string, JointTargets> joint_targets_;
    std::map<std::string, double> transition_q_start_;
    TransitionTarget transition_target_ = TARGET_REST;
    rclcpp::Time transition_start_time_;
    bool transition_time_initialized_ = false;

    // ─── Safety state ─────────────────────────────────────────────────
    SafetyState safety_state_ = SafetyState::SAFETY_NORMAL;
    std::vector<double> temp_ema_; // per motor joint
    std::vector<double> volt_ema_; // per distributor
    bool ema_initialized_ = false;
    int ema_warmup_counter_ = 0; // counts up to ema_window_samples_ before evaluating
    int warn_throttle_counter_ = 0;

    // Damping state
    bool damping_time_initialized_ = false;
    rclcpp::Time damping_start_time_;
    std::map<std::string, double> damping_q_start_;

    // Joints reference timeout
    rclcpp::Time last_joints_reference_time_;
    bool joints_reference_received_ = false;
    int joints_reference_timeout_throttle_ = 0;

    // NUC heartbeat monitoring
    rclcpp::Time last_heartbeat_time_;
    bool heartbeat_received_ = false;

    // ─── Buffered commands (protected by mutex) ─────────────────────────
    std::mutex var_mutex_;
    double base_vel_[3] = {0.0, 0.0, 0.0};
    double base_vel_filtered_[3] = {0.0, 0.0, 0.0};

    // Low-pass filter on velocity command
    bool lpf_enabled_ = false;
    double lpf_cutoff_freq_ = 1.0; // Hz
    double lpf_alpha_ = 0.0;       // Computed LPF coefficient
    rclcpp::Time last_vel_filter_time_;
    bool vel_filter_time_initialized_ = false;

    // Direct wheel commands: per-wheel buffered values
    std::map<std::string, double> direct_wheel_vel_cmd_;
    std::map<std::string, double> direct_wheel_kp_cmd_;
    std::map<std::string, double> direct_wheel_kd_cmd_;

    // Leg commands: per-joint buffered values
    std::map<std::string, double> leg_pos_cmd_;
    std::map<std::string, double> leg_vel_cmd_;
    std::map<std::string, double> leg_eff_cmd_;
    std::map<std::string, double> leg_kp_cmd_;
    std::map<std::string, double> leg_kd_cmd_;

    // ─── Interface index maps ───────────────────────────────────────────
    // Built in on_activate() by iterating command/state_interfaces_
    // Key: "joint_name/interface_name"
    std::map<std::string, size_t> cmd_idx_;
    std::map<std::string, size_t> stt_idx_;

    // ─── Publishers ─────────────────────────────────────────────────────
    rclcpp::Publisher<JointsStates>::SharedPtr stt_pub_;
    rclcpp::Publisher<JointsCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<PacketPass>::SharedPtr per_pub_;
    rclcpp::Publisher<DistributorsState>::SharedPtr dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr safety_pub_;

    // Pre-allocated messages
    JointsStates stt_msg_;
    JointsCommand cmd_msg_;
    PacketPass per_msg_;
    DistributorsState dist_msg_;
    geometry_msgs::msg::TwistStamped odom_msg_;

    // ─── Subscribers ────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<JointsCommand>::SharedPtr joints_reference_sub_;
    rclcpp::Subscription<JointsCommand>::SharedPtr direct_wheels_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;

    // ─── Services ───────────────────────────────────────────────────────
    rclcpp::Service<TransactionService>::SharedPtr activate_srv_;
    rclcpp::Service<TransactionService>::SharedPtr emergency_srv_;
    rclcpp::Service<TransactionService>::SharedPtr rest_srv_;
    rclcpp::Service<TransactionService>::SharedPtr stand_srv_;
    rclcpp::Service<TransactionService>::SharedPtr wheel_mode_srv_;

    // ─── Callbacks ──────────────────────────────────────────────────────
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void joints_reference_callback(const JointsCommand::SharedPtr msg);
    void activate_service_cb(
        const TransactionService::Request::SharedPtr req,
        const TransactionService::Response::SharedPtr res
    );
    void emergency_service_cb(
        const TransactionService::Request::SharedPtr req,
        const TransactionService::Response::SharedPtr res
    );
    void rest_service_cb(
        const TransactionService::Request::SharedPtr req,
        const TransactionService::Response::SharedPtr res
    );
    void stand_service_cb(
        const TransactionService::Request::SharedPtr req,
        const TransactionService::Response::SharedPtr res
    );
    void direct_wheels_callback(const JointsCommand::SharedPtr msg);
    void heartbeat_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void wheel_mode_service_cb(
        const TransactionService::Request::SharedPtr req,
        const TransactionService::Response::SharedPtr res
    );

    // ─── Update helpers ─────────────────────────────────────────────────
    void publish_joint_states(const rclcpp::Time& time);
    void publish_joints_command(const rclcpp::Time& time);
    void publish_performance(const rclcpp::Time& time);
    void publish_distributor_states(const rclcpp::Time& time);
    void publish_odometry(const rclcpp::Time& time);
    void write_wheel_commands();
    void write_direct_wheel_commands();
    void write_leg_commands();
    void zero_all_commands();
    void update_transition(const rclcpp::Time& time);
    void update_safety_monitoring();
    void evaluate_safety_transitions();
    void update_damping(const rclcpp::Time& time);
    void apply_velocity_filter(const rclcpp::Time& time);
    static double cosine_interp(double a, double b, double t);

    // ─── Helpers ────────────────────────────────────────────────────────
    /// Get state interface value by "joint/interface" key. Returns 0.0 if not found.
    double get_state(const std::string& key) const;
    /// Get command interface value by "joint/interface" key. Returns 0.0 if not found.
    double get_command(const std::string& key) const;
    /// Set command interface value by "joint/interface" key.
    void set_command(const std::string& key, double value);
};

} // namespace omni_controller

#endif // OMNI_CONTROLLER_HPP
