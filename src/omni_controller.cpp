#include "omni_controller/omni_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace omni_controller {

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn OmniController::on_init()
{
    try {
        // Wheel joint positions (map: position_key → joint name(s))
        // Mecanum: LF, LH, RF, RH (single string each)
        // Differential: LEFT, RIGHT (string array each — supports multiple wheels per side)
        auto_declare<std::string>("wheel_joints.LF", "");
        auto_declare<std::string>("wheel_joints.LH", "");
        auto_declare<std::string>("wheel_joints.RF", "");
        auto_declare<std::string>("wheel_joints.RH", "");
        auto_declare<std::vector<std::string>>("wheel_joints.LEFT", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("wheel_joints.RIGHT", std::vector<std::string>());

        // Other joint lists
        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("distributor_names", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("second_encoder_joints", std::vector<std::string>());

        // Wheel IK params
        auto_declare<std::string>("feet_type", "none");
        auto_declare<double>("wheel_rad", 0.05);
        auto_declare<double>("driveshaft_x", 0.235);
        auto_declare<double>("driveshaft_y", 0.188);
        auto_declare<double>("mecanum_angle", 135.0);
        auto_declare<double>("track_width", 0.4);

        // Communication
        auto_declare<int>("input_frequency", 100);
        auto_declare<bool>("BestEffort_QOS", true);

        // Feature flags
        auto_declare<bool>("sim", false);
        auto_declare<double>("sim_kp", 50.0);
        auto_declare<double>("sim_kd", 1.0);
        auto_declare<bool>("pub_odom", false);
        auto_declare<bool>("pub_performance", true);
        auto_declare<std::string>("wheel_mode", "ik");

        // Transitions (rest / stand)
        auto_declare<double>("rest_duration", 0.0);
        auto_declare<double>("stand_duration", 0.0);

        // Safety
        auto_declare<bool>("safety.enabled", true);
        auto_declare<double>("safety.temp_warning_threshold", 50.0);
        auto_declare<double>("safety.temp_critical_threshold", 57.0);
        auto_declare<double>("safety.battery_min_voltage", 24.0);
        auto_declare<double>("safety.temp_recovery_hysteresis", 5.0);
        auto_declare<double>("safety.volt_recovery_hysteresis", 1.0);
        auto_declare<int>("safety.ema_window_samples", 5000);
        auto_declare<std::string>("safety.critical_strategy", "damping");
        auto_declare<double>("safety.damping_duration", 3.0);
        auto_declare<double>("safety.joints_reference_timeout", 0.5);
        auto_declare<double>("safety.heartbeat_timeout", 1.0);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Exception during parameter declaration: %s", e.what()
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "on_init successful");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniController::on_configure(const rclcpp_lifecycle::State&)
{
    // ── Read parameters ─────────────────────────────────────────────────
    std::string feet_type = get_node()->get_parameter("feet_type").as_string();

    // Build wheel_joints_ (flat list) and wheel_groups_ (IK index → joints)
    wheel_joints_.clear();
    wheel_groups_.clear();
    if (feet_type == "mecanum") {
        const std::vector<std::string> positions = {"LF", "LH", "RF", "RH"};
        for (const auto& pos : positions) {
            std::string jnt = get_node()->get_parameter("wheel_joints." + pos).as_string();
            if (jnt.empty()) {
                RCLCPP_ERROR(
                    get_node()->get_logger(), "wheel_joints.%s is required for feet_type='mecanum'",
                    pos.c_str()
                );
                return CallbackReturn::ERROR;
            }
            wheel_joints_.push_back(jnt);
            wheel_groups_.push_back({jnt});
        }
    } else if (feet_type == "differential") {
        const std::vector<std::string> positions = {"LEFT", "RIGHT"};
        for (const auto& pos : positions) {
            auto joints = get_node()->get_parameter("wheel_joints." + pos).as_string_array();
            if (joints.empty()) {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "wheel_joints.%s requires at least one joint for feet_type='differential'",
                    pos.c_str()
                );
                return CallbackReturn::ERROR;
            }
            wheel_groups_.push_back(joints);
            wheel_joints_.insert(wheel_joints_.end(), joints.begin(), joints.end());
        }
    }

    joints_ = get_node()->get_parameter("joints").as_string_array();
    distributor_names_ = get_node()->get_parameter("distributor_names").as_string_array();
    second_encoder_joints_ = get_node()->get_parameter("second_encoder_joints").as_string_array();
    sim_flag_ = get_node()->get_parameter("sim").as_bool();
    if (sim_flag_) {
        sim_kp_ = get_node()->get_parameter("sim_kp").as_double();
        sim_kd_ = get_node()->get_parameter("sim_kd").as_double();
    }
    pub_odom_ = get_node()->get_parameter("pub_odom").as_bool();
    pub_performance_ = get_node()->get_parameter("pub_performance").as_bool();

    has_wheels_ = !wheel_joints_.empty();
    has_legs_ = !joints_.empty();
    has_distributors_ = !distributor_names_.empty();

    // ── Build combined motor joint list ─────────────────────────────────
    all_motor_joints_.clear();
    all_motor_joints_.insert(all_motor_joints_.end(), wheel_joints_.begin(), wheel_joints_.end());
    all_motor_joints_.insert(all_motor_joints_.end(), joints_.begin(), joints_.end());

    // Build second-encoder flags per motor joint
    se_flag_.resize(all_motor_joints_.size(), false);
    for (size_t i = 0; i < all_motor_joints_.size(); i++) {
        for (const auto& se : second_encoder_joints_) {
            if (all_motor_joints_[i] == se) {
                se_flag_[i] = true;
                break;
            }
        }
    }

    // ── Validate and create WheelIK ─────────────────────────────────────
    if (has_wheels_) {
        try {
            wheel_ik_ = create_wheel_ik(feet_type);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "WheelIK creation failed: %s", e.what());
            return CallbackReturn::ERROR;
        }

        if (wheel_ik_) {
            WheelIKConfig ik_config;
            ik_config.wheel_rad = get_node()->get_parameter("wheel_rad").as_double();
            ik_config.driveshaft_x = get_node()->get_parameter("driveshaft_x").as_double();
            ik_config.driveshaft_y = get_node()->get_parameter("driveshaft_y").as_double();
            ik_config.mecanum_angle = get_node()->get_parameter("mecanum_angle").as_double();
            ik_config.track_width = get_node()->get_parameter("track_width").as_double();

            try {
                // For differential with multi-wheel groups, pass one name per group
                // (IK only cares about count, not the actual names)
                std::vector<std::string> ik_names;
                for (const auto& group : wheel_groups_)
                    ik_names.push_back(group.front());
                wheel_ik_->configure(ik_config, ik_names);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_node()->get_logger(), "WheelIK configure failed: %s", e.what());
                return CallbackReturn::ERROR;
            }
        } else if (feet_type == "none") {
            RCLCPP_WARN(
                get_node()->get_logger(), "wheel_joints is non-empty but feet_type='none'. Wheels "
                                          "will not receive IK commands."
            );
        }
    }

    // ── Initialize leg command buffers ───────────────────────────────────
    for (const auto& jnt : joints_) {
        leg_pos_cmd_[jnt] = 0.0;
        leg_vel_cmd_[jnt] = 0.0;
        leg_eff_cmd_[jnt] = 0.0;
        leg_kp_cmd_[jnt] = 1.0;
        leg_kd_cmd_[jnt] = 1.0;
    }

    // ── Wheel mode ──────────────────────────────────────────────────────
    std::string wheel_mode_str = get_node()->get_parameter("wheel_mode").as_string();
    if (wheel_mode_str == "direct")
        wheel_mode_ = WheelMode::WHEEL_DIRECT;
    else
        wheel_mode_ = WheelMode::WHEEL_IK;

    // Initialize direct wheel command buffers
    for (const auto& jnt : wheel_joints_) {
        direct_wheel_vel_cmd_[jnt] = 0.0;
        direct_wheel_kp_cmd_[jnt] = 0.0;
        direct_wheel_kd_cmd_[jnt] = 1.0;
    }

    // ── Transition configuration (rest / stand) ──────────────────────────
    rest_duration_ = get_node()->get_parameter("rest_duration").as_double();
    stand_duration_ = get_node()->get_parameter("stand_duration").as_double();
    if ((rest_duration_ <= 0.0 && stand_duration_ <= 0.0) || !has_legs_) {
        has_transitions_ = false;
    } else {
        has_transitions_ = true;
        for (const auto& jnt : joints_) {
            auto_declare<double>("joint_targets." + jnt + ".q_rest", 0.0);
            auto_declare<double>("joint_targets." + jnt + ".q_stand", 0.0);

            JointTargets cfg;
            cfg.q_rest = get_node()->get_parameter("joint_targets." + jnt + ".q_rest").as_double();
            cfg.q_stand =
                get_node()->get_parameter("joint_targets." + jnt + ".q_stand").as_double();
            joint_targets_[jnt] = cfg;
        }
        RCLCPP_INFO(
            get_node()->get_logger(), "Transitions configured for %zu leg joints", joints_.size()
        );
    }

    // ── Safety parameters ─────────────────────────────────────────────
    safety_enabled_ = get_node()->get_parameter("safety.enabled").as_bool();
    temp_warning_threshold_ =
        get_node()->get_parameter("safety.temp_warning_threshold").as_double();
    temp_critical_threshold_ =
        get_node()->get_parameter("safety.temp_critical_threshold").as_double();
    battery_min_voltage_ = get_node()->get_parameter("safety.battery_min_voltage").as_double();
    temp_recovery_hysteresis_ =
        get_node()->get_parameter("safety.temp_recovery_hysteresis").as_double();
    volt_recovery_hysteresis_ =
        get_node()->get_parameter("safety.volt_recovery_hysteresis").as_double();
    ema_window_samples_ = get_node()->get_parameter("safety.ema_window_samples").as_int();
    critical_strategy_ = get_node()->get_parameter("safety.critical_strategy").as_string();
    damping_duration_ = get_node()->get_parameter("safety.damping_duration").as_double();
    joints_reference_timeout_ =
        get_node()->get_parameter("safety.joints_reference_timeout").as_double();
    heartbeat_timeout_ = get_node()->get_parameter("safety.heartbeat_timeout").as_double();

    if (critical_strategy_ != "damping" && critical_strategy_ != "default_config" &&
        critical_strategy_ != "stop") {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Invalid safety.critical_strategy '%s', must be 'damping', 'default_config', or 'stop'",
            critical_strategy_.c_str()
        );
        return CallbackReturn::ERROR;
    }

    // Compute EMA alpha: alpha = 2 / (N + 1)
    ema_alpha_ = 2.0 / (static_cast<double>(ema_window_samples_) + 1.0);

    // Resize EMA arrays
    temp_ema_.resize(all_motor_joints_.size(), 0.0);
    volt_ema_.resize(distributor_names_.size(), 0.0);

    // Declare default_config parameters for each leg joint (fallback to q_rest = 0.0)
    if (critical_strategy_ == "default_config") {
        for (const auto& jnt : joints_) {
            double default_q = 0.0;
            if (joint_targets_.count(jnt))
                default_q = joint_targets_[jnt].q_rest;
            auto_declare<double>("safety.default_config." + jnt + ".q", default_q);
        }
    }

    // ── QoS setup ───────────────────────────────────────────────────────
    bool best_effort = get_node()->get_parameter("BestEffort_QOS").as_bool();
    int input_freq = get_node()->get_parameter("input_frequency").as_int();

    // ── Subscribers ─────────────────────────────────────────────────────
    if (has_wheels_ && wheel_ik_) {
        rclcpp::QoS twist_qos(10);
        if (best_effort)
            twist_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        milliseconds deadline_dur{input_freq + 5};
        twist_qos.deadline(deadline_dur);

        rclcpp::SubscriptionOptions sub_opt;
        sub_opt.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo&) {
            dl_miss_count_++;
            if (dl_miss_count_ > 10) {
                std::lock_guard<std::mutex> lg(var_mutex_);
                if (wheel_mode_ == WHEEL_IK)
                    c_stt_ = ControllerState::INACTIVE;
            }
            RCLCPP_WARN(get_node()->get_logger(), "Twist deadline missed");
        };

        twist_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "~/twist_cmd", twist_qos,
            std::bind(&OmniController::twist_callback, this, std::placeholders::_1), sub_opt
        );
    }

    if (has_legs_) {
        rclcpp::QoS joints_reference_qos(5);
        joints_reference_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

        joints_reference_sub_ = get_node()->create_subscription<JointsCommand>(
            "~/joints_reference", joints_reference_qos,
            std::bind(&OmniController::joints_reference_callback, this, std::placeholders::_1)
        );
    }

    if (has_wheels_) {
        rclcpp::QoS direct_qos(5);
        direct_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);

        direct_wheels_sub_ = get_node()->create_subscription<JointsCommand>(
            "~/direct_wheels_cmd", direct_qos,
            std::bind(&OmniController::direct_wheels_callback, this, std::placeholders::_1)
        );
    }

    // Heartbeat subscriber (BestEffort, depth 1)
    if (safety_enabled_ && heartbeat_timeout_ > 0.0) {
        rclcpp::QoS hb_qos(1);
        hb_qos.best_effort();
        heartbeat_sub_ = get_node()->create_subscription<std_msgs::msg::Empty>(
            "/nuc_heartbeat", hb_qos,
            std::bind(&OmniController::heartbeat_callback, this, std::placeholders::_1)
        );
    }

    // ── Publishers ──────────────────────────────────────────────────────
    stt_pub_ = get_node()->create_publisher<JointsStates>("~/joints_state", 5);
    cmd_pub_ = get_node()->create_publisher<JointsCommand>("~/debug/joints_command", 5);

    if (pub_performance_)
        per_pub_ = get_node()->create_publisher<PacketPass>("~/performance", 5);

    if (has_distributors_)
        dist_pub_ = get_node()->create_publisher<DistributorsState>("~/distributors_state", 5);

    if (pub_odom_ && has_wheels_ && wheel_ik_)
        odom_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("~/odom", 10);

    if (safety_enabled_)
        safety_pub_ = get_node()->create_publisher<std_msgs::msg::UInt8>("~/safety_state", 5);

    // ── Services ────────────────────────────────────────────────────────
    activate_srv_ = get_node()->create_service<TransactionService>(
        "~/activate_srv",
        std::bind(
            &OmniController::activate_service_cb, this, std::placeholders::_1, std::placeholders::_2
        )
    );

    emergency_srv_ = get_node()->create_service<TransactionService>(
        "~/emergency_srv", std::bind(
                               &OmniController::emergency_service_cb, this, std::placeholders::_1,
                               std::placeholders::_2
                           )
    );

    if (has_transitions_) {
        rest_srv_ = get_node()->create_service<TransactionService>(
            "~/rest_srv",
            std::bind(
                &OmniController::rest_service_cb, this, std::placeholders::_1, std::placeholders::_2
            )
        );
        stand_srv_ = get_node()->create_service<TransactionService>(
            "~/stand_srv", std::bind(
                               &OmniController::stand_service_cb, this, std::placeholders::_1,
                               std::placeholders::_2
                           )
        );
    }

    if (has_wheels_) {
        wheel_mode_srv_ = get_node()->create_service<TransactionService>(
            "~/set_wheel_mode", std::bind(
                                    &OmniController::wheel_mode_service_cb, this,
                                    std::placeholders::_1, std::placeholders::_2
                                )
        );
    }

    // ── Pre-allocate messages ───────────────────────────────────────────
    size_t n_motors = all_motor_joints_.size();

    stt_msg_.name.resize(n_motors);
    stt_msg_.position.resize(n_motors);
    stt_msg_.velocity.resize(n_motors);
    stt_msg_.effort.resize(n_motors);
    stt_msg_.temperature.resize(n_motors);
    stt_msg_.current.resize(n_motors);

    // Only allocate sec_enc arrays if any secondary encoders exist
    bool has_sec_enc = std::any_of(se_flag_.begin(), se_flag_.end(), [](bool v) { return v; });
    if (has_sec_enc) {
        stt_msg_.sec_enc_pos.resize(n_motors, 0.0);
        stt_msg_.sec_enc_vel.resize(n_motors, 0.0);
    }

    cmd_msg_.name.resize(n_motors);
    cmd_msg_.position.resize(n_motors);
    cmd_msg_.velocity.resize(n_motors);
    cmd_msg_.effort.resize(n_motors);
    cmd_msg_.kp_scale.resize(n_motors);
    cmd_msg_.kd_scale.resize(n_motors);
    for (size_t i = 0; i < n_motors; i++)
        cmd_msg_.name[i] = all_motor_joints_[i];

    if (pub_performance_) {
        per_msg_.name.resize(n_motors);
        per_msg_.pack_loss.resize(n_motors);
        for (size_t i = 0; i < n_motors; i++)
            per_msg_.name[i] = all_motor_joints_[i];
    }

    if (has_distributors_) {
        size_t n_dist = distributor_names_.size();
        dist_msg_.name.resize(n_dist);
        dist_msg_.voltage.resize(n_dist);
        dist_msg_.current.resize(n_dist);
        dist_msg_.temperature.resize(n_dist);
    }

    RCLCPP_INFO(
        get_node()->get_logger(), "on_configure successful (wheels=%zu, legs=%zu, dist=%zu)",
        wheel_joints_.size(), joints_.size(), distributor_names_.size()
    );
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniController::on_activate(const rclcpp_lifecycle::State&)
{
    // Build command index map
    cmd_idx_.clear();
    for (size_t i = 0; i < command_interfaces_.size(); i++) {
        std::string key = command_interfaces_[i].get_prefix_name() + "/" +
                          command_interfaces_[i].get_interface_name();
        cmd_idx_[key] = i;
    }

    // Build state index map
    stt_idx_.clear();
    for (size_t i = 0; i < state_interfaces_.size(); i++) {
        std::string key = state_interfaces_[i].get_prefix_name() + "/" +
                          state_interfaces_[i].get_interface_name();
        stt_idx_[key] = i;
    }

    // Start in INACTIVE — wait for activate_srv call
    c_stt_ = ControllerState::INACTIVE;
    dl_miss_count_ = 0;

    // Reset safety state
    safety_state_ = SafetyState::SAFETY_NORMAL;
    ema_initialized_ = false;
    ema_warmup_counter_ = 0;
    warn_throttle_counter_ = 0;
    damping_time_initialized_ = false;
    joints_reference_received_ = false;
    joints_reference_timeout_throttle_ = 0;
    heartbeat_received_ = false;

    RCLCPP_INFO(
        get_node()->get_logger(), "on_activate successful (INACTIVE, waiting for activate_srv)"
    );
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniController::on_deactivate(const rclcpp_lifecycle::State&)
{
    c_stt_ = ControllerState::INACTIVE;
    zero_all_commands();
    RCLCPP_INFO(get_node()->get_logger(), "on_deactivate");
    return CallbackReturn::SUCCESS;
}

CallbackReturn OmniController::on_cleanup(const rclcpp_lifecycle::State&)
{
    return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Interface Configuration
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::InterfaceConfiguration OmniController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    if (sim_flag_) {
        // Simulation: wheels use velocity, legs use effort
        for (const auto& jnt : wheel_joints_)
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
        for (const auto& jnt : joints_)
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_EFFORT);
    } else {
        // Real hardware: all joints use velocity + position + effort + kp_scale + kd_scale
        for (const auto& jnt : all_motor_joints_) {
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_POSITION);
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_EFFORT);
            cfg.names.push_back(jnt + "/" + hw_if::KP_SCALE);
            cfg.names.push_back(jnt + "/" + hw_if::KD_SCALE);
        }
    }
    return cfg;
}

controller_interface::InterfaceConfiguration OmniController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    if (sim_flag_) {
        // Simulation: only standard state interfaces per motor joint
        for (const auto& jnt : all_motor_joints_) {
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_POSITION);
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_EFFORT);
        }
    } else {
        // Real hardware: full state interfaces

        // System-level performance interfaces
        std::string sys_name = "Pi3Hat";
        cfg.names.push_back(sys_name + "/" + hw_if::VALIDITY_LOSS);
        cfg.names.push_back(sys_name + "/" + hw_if::CYCLE_DUR);

        // Per-motor package loss
        for (const auto& jnt : all_motor_joints_)
            cfg.names.push_back(jnt + "/" + hw_if::PACKAGE_LOSS);

        // Per-motor state
        for (const auto& jnt : all_motor_joints_) {
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_POSITION);
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
            cfg.names.push_back(jnt + "/" + hardware_interface::HW_IF_EFFORT);
            cfg.names.push_back(jnt + "/" + hw_if::TEMPERATURE);
            cfg.names.push_back(jnt + "/" + hw_if::Q_CURRENT);
        }

        // Second encoder state
        for (size_t i = 0; i < all_motor_joints_.size(); i++) {
            if (se_flag_[i]) {
                cfg.names.push_back(
                    all_motor_joints_[i] + "_second_encoder/" + hardware_interface::HW_IF_POSITION
                );
                cfg.names.push_back(
                    all_motor_joints_[i] + "_second_encoder/" + hardware_interface::HW_IF_VELOCITY
                );
            }
        }

        // Distributor state
        for (const auto& dist : distributor_names_) {
            cfg.names.push_back(dist + "/" + hw_if::VOLTAGE);
            cfg.names.push_back(dist + "/" + hw_if::CURRENT);
            cfg.names.push_back(dist + "/" + hw_if::TEMPERATURE);
        }
    }

    return cfg;
}

// ═══════════════════════════════════════════════════════════════════════════
//  update() — main control loop
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::return_type
OmniController::update(const rclcpp::Time& time, const rclcpp::Duration&)
{
    // Phase 1: Read and publish state
    publish_joint_states(time);
    if (!sim_flag_ && pub_performance_ && per_pub_)
        publish_performance(time);
    if (!sim_flag_ && has_distributors_ && dist_pub_)
        publish_distributor_states(time);
    if (pub_odom_ && has_wheels_ && wheel_ik_ && odom_pub_)
        publish_odometry(time);

    // Phase 2: Process commands (with safety override)
    std::lock_guard<std::mutex> lg(var_mutex_);

    // Safety monitoring (inside mutex — safety_state_ is read by service callbacks)
    if (safety_enabled_)
        update_safety_monitoring();

    // Publish safety state
    if (safety_enabled_ && safety_pub_) {
        std_msgs::msg::UInt8 safety_msg;
        safety_msg.data = static_cast<uint8_t>(safety_state_);
        safety_pub_->publish(safety_msg);
    }

    // Safety override: CRITICAL / DAMPING / STOPPED take precedence
    bool safety_handled = false;
    if (safety_enabled_) {
        switch (safety_state_) {
        case SafetyState::SAFETY_CRITICAL:
            // Force inactive, then transition to DAMPING or STOPPED
            c_stt_ = ControllerState::INACTIVE;
            if (critical_strategy_ == "stop") {
                safety_state_ = SafetyState::SAFETY_STOPPED;
                zero_all_commands();
                RCLCPP_ERROR(get_node()->get_logger(), "SAFETY CRITICAL: motors stopped");
            } else {
                safety_state_ = SafetyState::SAFETY_DAMPING;
                damping_time_initialized_ = false;
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "SAFETY CRITICAL: starting damping sequence (strategy: %s)",
                    critical_strategy_.c_str()
                );
                update_damping(time);
            }
            safety_handled = true;
            break;

        case SafetyState::SAFETY_DAMPING:
            c_stt_ = ControllerState::INACTIVE;
            update_damping(time);
            safety_handled = true;
            break;

        case SafetyState::SAFETY_STOPPED:
            c_stt_ = ControllerState::INACTIVE;
            zero_all_commands();
            safety_handled = true;
            break;

        default:
            break; // NORMAL / WARNING: continue with normal state machine
        }
    }

    // Normal state machine (skipped when safety override is active)
    if (!safety_handled) {
        switch (c_stt_) {
        case ControllerState::INACTIVE:
            zero_all_commands();
            break;
        case ControllerState::TRANSITION:
            update_transition(time);
            break;
        case ControllerState::ACTIVE:
            if (has_wheels_) {
                if (wheel_mode_ == WHEEL_IK && wheel_ik_)
                    write_wheel_commands();
                else if (wheel_mode_ == WHEEL_DIRECT)
                    write_direct_wheel_commands();
            }
            if (has_legs_) {
                // Check joints reference timeout
                if (joints_reference_received_) {
                    double since_last = (time - last_joints_reference_time_).seconds();
                    if (since_last > joints_reference_timeout_) {
                        // Hold last position — zero feedforward for pure position hold
                        for (const auto& jnt : joints_) {
                            leg_vel_cmd_[jnt] = 0.0;
                            leg_eff_cmd_[jnt] = 0.0;
                        }
                        if (++joints_reference_timeout_throttle_ >= 500) {
                            RCLCPP_WARN(
                                get_node()->get_logger(),
                                "Joints reference timeout (%.2fs since last reference), holding "
                                "position",
                                since_last
                            );
                            joints_reference_timeout_throttle_ = 0;
                        }
                    }
                }
                write_leg_commands();
            }
            break;
        }
    }

    // Phase 3: Publish actual commands being sent to hardware
    publish_joints_command(time);

    return controller_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Callbacks
// ═══════════════════════════════════════════════════════════════════════════

void OmniController::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    dl_miss_count_ = 0;
    if (c_stt_ == ControllerState::ACTIVE) {
        base_vel_[0] = msg->linear.x;
        base_vel_[1] = msg->linear.y;
        base_vel_[2] = msg->angular.z;
    }
}

void OmniController::heartbeat_callback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    last_heartbeat_time_ = get_node()->now();
    heartbeat_received_ = true;
}

void OmniController::joints_reference_callback(const JointsCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (c_stt_ == ControllerState::TRANSITION)
        return;
    last_joints_reference_time_ = get_node()->now();
    joints_reference_received_ = true;
    joints_reference_timeout_throttle_ = 0;
    for (size_t i = 0; i < msg->name.size(); i++) {
        const auto& name = msg->name[i];
        if (leg_pos_cmd_.count(name)) {
            if (i < msg->position.size())
                leg_pos_cmd_[name] = msg->position[i];
            if (i < msg->velocity.size())
                leg_vel_cmd_[name] = msg->velocity[i];
            if (i < msg->effort.size())
                leg_eff_cmd_[name] = msg->effort[i];
            if (i < msg->kp_scale.size())
                leg_kp_cmd_[name] = msg->kp_scale[i];
            if (i < msg->kd_scale.size())
                leg_kd_cmd_[name] = msg->kd_scale[i];
        }
    }
}

void OmniController::activate_service_cb(
    const TransactionService::Request::SharedPtr req,
    const TransactionService::Response::SharedPtr res
)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (safety_enabled_ && safety_state_ != SafetyState::SAFETY_NORMAL) {
        res->success = false;
        res->message = "Cannot activate: safety state is not NORMAL";
        return;
    }
    if (c_stt_ == ControllerState::INACTIVE && req->data) {
        // Snap leg commands to actual positions to prevent jumps
        if (has_legs_) {
            for (const auto& jnt : joints_) {
                leg_pos_cmd_[jnt] = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
                leg_vel_cmd_[jnt] = 0.0;
                leg_eff_cmd_[jnt] = 0.0;
                leg_kp_cmd_[jnt] = 1.0;
                leg_kd_cmd_[jnt] = 1.0;
            }
        }
        c_stt_ = ControllerState::ACTIVE;
        res->success = true;
        res->message = "Active mode activated";
    } else {
        res->success = false;
        res->message = req->data ? "Not in Inactive mode" : "Invalid request";
    }
}

void OmniController::direct_wheels_callback(const JointsCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (c_stt_ != ControllerState::ACTIVE)
        return;
    for (size_t i = 0; i < msg->name.size(); i++) {
        const auto& name = msg->name[i];
        if (direct_wheel_vel_cmd_.count(name)) {
            if (i < msg->velocity.size())
                direct_wheel_vel_cmd_[name] = msg->velocity[i];
            if (i < msg->kp_scale.size())
                direct_wheel_kp_cmd_[name] = msg->kp_scale[i];
            if (i < msg->kd_scale.size())
                direct_wheel_kd_cmd_[name] = msg->kd_scale[i];
        }
    }
}

void OmniController::wheel_mode_service_cb(
    const TransactionService::Request::SharedPtr req,
    const TransactionService::Response::SharedPtr res
)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (!has_wheels_) {
        res->success = false;
        res->message = "No wheels configured";
        return;
    }
    wheel_mode_ = req->data ? WHEEL_DIRECT : WHEEL_IK;
    dl_miss_count_ = 0;
    // Zero both command buffers on mode switch
    base_vel_[0] = 0.0;
    base_vel_[1] = 0.0;
    base_vel_[2] = 0.0;
    for (auto& [name, vel] : direct_wheel_vel_cmd_)
        vel = 0.0;
    res->success = true;
    res->message =
        (wheel_mode_ == WHEEL_DIRECT) ? "Switched to direct mode" : "Switched to IK mode";
    RCLCPP_INFO(get_node()->get_logger(), "%s", res->message.c_str());
}

void OmniController::emergency_service_cb(
    const TransactionService::Request::SharedPtr req,
    const TransactionService::Response::SharedPtr res
)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (c_stt_ != ControllerState::INACTIVE && req->data) {
        c_stt_ = ControllerState::INACTIVE;
        res->success = true;
        res->message = "Emergency mode activated";
    } else {
        res->success = false;
        res->message = req->data ? "Already in Emergency mode" : "Invalid request";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  State publishing
// ═══════════════════════════════════════════════════════════════════════════

void OmniController::publish_joint_states(const rclcpp::Time& time)
{
    stt_msg_.header.set__stamp(time);
    for (size_t i = 0; i < all_motor_joints_.size(); i++) {
        const auto& jnt = all_motor_joints_[i];
        stt_msg_.name[i] = jnt;
        stt_msg_.position[i] = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
        stt_msg_.velocity[i] = get_state(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
        stt_msg_.effort[i] = get_state(jnt + "/" + hardware_interface::HW_IF_EFFORT);
        stt_msg_.temperature[i] = get_state(jnt + "/" + hw_if::TEMPERATURE);
        stt_msg_.current[i] = get_state(jnt + "/" + hw_if::Q_CURRENT);

        if (se_flag_[i]) {
            stt_msg_.sec_enc_pos[i] =
                get_state(jnt + "_second_encoder/" + hardware_interface::HW_IF_POSITION);
            stt_msg_.sec_enc_vel[i] =
                get_state(jnt + "_second_encoder/" + hardware_interface::HW_IF_VELOCITY);
        }
    }
    stt_pub_->publish(stt_msg_);
}

void OmniController::publish_joints_command(const rclcpp::Time& time)
{
    cmd_msg_.header.set__stamp(time);
    for (size_t i = 0; i < all_motor_joints_.size(); i++) {
        const auto& jnt = all_motor_joints_[i];
        cmd_msg_.position[i] = get_command(jnt + "/" + hardware_interface::HW_IF_POSITION);
        cmd_msg_.velocity[i] = get_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
        cmd_msg_.effort[i] = get_command(jnt + "/" + hardware_interface::HW_IF_EFFORT);
        cmd_msg_.kp_scale[i] = get_command(jnt + "/" + hw_if::KP_SCALE);
        cmd_msg_.kd_scale[i] = get_command(jnt + "/" + hw_if::KD_SCALE);
    }
    cmd_pub_->publish(cmd_msg_);
}

void OmniController::publish_performance(const rclcpp::Time& time)
{
    per_msg_.header.set__stamp(time);
    per_msg_.set__valid(get_state("Pi3Hat/" + std::string(hw_if::VALIDITY_LOSS)));
    per_msg_.set__cycle_dur(get_state("Pi3Hat/" + std::string(hw_if::CYCLE_DUR)));

    for (size_t i = 0; i < all_motor_joints_.size(); i++) {
        per_msg_.pack_loss[i] =
            get_state(all_motor_joints_[i] + "/" + std::string(hw_if::PACKAGE_LOSS));
    }
    per_pub_->publish(per_msg_);
}

void OmniController::publish_distributor_states(const rclcpp::Time& time)
{
    dist_msg_.header.set__stamp(time);
    for (size_t i = 0; i < distributor_names_.size(); i++) {
        const auto& name = distributor_names_[i];
        dist_msg_.name[i] = name;
        dist_msg_.voltage[i] = get_state(name + "/" + std::string(hw_if::VOLTAGE));
        dist_msg_.current[i] = get_state(name + "/" + std::string(hw_if::CURRENT));
        dist_msg_.temperature[i] = get_state(name + "/" + std::string(hw_if::TEMPERATURE));
    }
    dist_pub_->publish(dist_msg_);
}

void OmniController::publish_odometry(const rclcpp::Time& time)
{
    // Build one velocity per IK slot by averaging all wheels in each group
    std::vector<double> wheel_vels(wheel_groups_.size());
    for (size_t g = 0; g < wheel_groups_.size(); g++) {
        double sum = 0.0;
        for (const auto& jnt : wheel_groups_[g])
            sum += get_state(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
        wheel_vels[g] = sum / static_cast<double>(wheel_groups_[g].size());
    }

    auto odom = wheel_ik_->forward(wheel_vels);
    odom_msg_.header.set__stamp(time);
    odom_msg_.twist.linear.set__x(odom[0]);
    odom_msg_.twist.linear.set__y(odom[1]);
    odom_msg_.twist.linear.set__z(0.0);
    odom_msg_.twist.angular.set__x(0.0);
    odom_msg_.twist.angular.set__y(0.0);
    odom_msg_.twist.angular.set__z(odom[2]);
    odom_pub_->publish(odom_msg_);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Command writing
// ═══════════════════════════════════════════════════════════════════════════

void OmniController::write_wheel_commands()
{
    auto wheel_vels = wheel_ik_->inverse(base_vel_[0], base_vel_[1], base_vel_[2]);

    for (size_t g = 0; g < wheel_groups_.size(); g++) {
        for (const auto& jnt : wheel_groups_[g]) {
            set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, wheel_vels[g]);
            if (!sim_flag_) {
                set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, std::nan("1"));
                set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
                set_command(jnt + "/" + hw_if::KP_SCALE, 0.0);
                set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
            }
        }
    }
}

void OmniController::write_direct_wheel_commands()
{
    for (const auto& jnt : wheel_joints_) {
        set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, direct_wheel_vel_cmd_[jnt]);
        if (!sim_flag_) {
            set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, std::nan("1"));
            set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
            set_command(jnt + "/" + hw_if::KP_SCALE, direct_wheel_kp_cmd_[jnt]);
            set_command(jnt + "/" + hw_if::KD_SCALE, direct_wheel_kd_cmd_[jnt]);
        }
    }
}

void OmniController::write_leg_commands()
{
    if (sim_flag_) {
        // Simulation: compute effort via PD + feedforward
        for (const auto& jnt : joints_) {
            double pos_actual = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
            double vel_actual = get_state(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
            double tau = sim_kp_ * (leg_pos_cmd_[jnt] - pos_actual)
                       + sim_kd_ * (leg_vel_cmd_[jnt] - vel_actual)
                       + leg_eff_cmd_[jnt];
            set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, tau);
        }
    } else {
        for (const auto& jnt : joints_) {
            set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, leg_pos_cmd_[jnt]);
            set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, leg_vel_cmd_[jnt]);
            set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, leg_eff_cmd_[jnt]);
            set_command(jnt + "/" + hw_if::KP_SCALE, leg_kp_cmd_[jnt]);
            set_command(jnt + "/" + hw_if::KD_SCALE, leg_kd_cmd_[jnt]);
        }
    }
}

void OmniController::zero_all_commands()
{
    if (sim_flag_) {
        for (const auto& jnt : wheel_joints_)
            set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
        for (const auto& jnt : joints_)
            set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
    } else {
        for (const auto& jnt : all_motor_joints_) {
            set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
            set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, std::nan("1"));
            set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
            set_command(jnt + "/" + hw_if::KP_SCALE, 0.0);
            set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Transitions (rest / stand)
// ═══════════════════════════════════════════════════════════════════════════

void OmniController::rest_service_cb(
    const TransactionService::Request::SharedPtr req,
    const TransactionService::Response::SharedPtr res
)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (safety_enabled_ && safety_state_ != SafetyState::SAFETY_NORMAL) {
        res->success = false;
        res->message = "Cannot start rest: safety state is not NORMAL";
        return;
    }
    if ((c_stt_ == ControllerState::INACTIVE || c_stt_ == ControllerState::ACTIVE) &&
        has_transitions_ && req->data) {
        transition_target_ = TARGET_REST;
        transition_time_initialized_ = false;
        c_stt_ = ControllerState::TRANSITION;
        res->success = true;
        res->message = "Rest transition started";
        RCLCPP_INFO(get_node()->get_logger(), "Transition started (target: rest)");
    } else {
        res->success = false;
        res->message = req->data ? "Cannot start rest (already transitioning or not configured)"
                                 : "Invalid request";
    }
}

void OmniController::stand_service_cb(
    const TransactionService::Request::SharedPtr req,
    const TransactionService::Response::SharedPtr res
)
{
    std::lock_guard<std::mutex> lg(var_mutex_);
    if (safety_enabled_ && safety_state_ != SafetyState::SAFETY_NORMAL) {
        res->success = false;
        res->message = "Cannot start stand: safety state is not NORMAL";
        return;
    }
    if ((c_stt_ == ControllerState::INACTIVE || c_stt_ == ControllerState::ACTIVE) &&
        has_transitions_ && req->data) {
        transition_target_ = TARGET_STAND;
        transition_time_initialized_ = false;
        c_stt_ = ControllerState::TRANSITION;
        res->success = true;
        res->message = "Stand transition started";
        RCLCPP_INFO(get_node()->get_logger(), "Transition started (target: stand)");
    } else {
        res->success = false;
        res->message = req->data ? "Cannot start stand (already transitioning or not configured)"
                                 : "Invalid request";
    }
}

double OmniController::cosine_interp(double a, double b, double t)
{
    return a + 0.5 * (1.0 - std::cos(M_PI * t)) * (b - a);
}

void OmniController::update_transition(const rclcpp::Time& time)
{
    // Initialize timer on first call
    if (!transition_time_initialized_) {
        transition_start_time_ = time;
        transition_time_initialized_ = true;

        // Read current joint positions as the actual origin
        for (const auto& jnt : joints_)
            transition_q_start_[jnt] = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
    }

    double duration = (transition_target_ == TARGET_REST) ? rest_duration_ : stand_duration_;
    double elapsed = (time - transition_start_time_).seconds();
    double t = std::clamp(elapsed / duration, 0.0, 1.0);

    // Interpolate leg joints: current_pos → target
    for (const auto& jnt : joints_) {
        const auto& cfg = joint_targets_[jnt];
        double q_target = (transition_target_ == TARGET_REST) ? cfg.q_rest : cfg.q_stand;
        double q = cosine_interp(transition_q_start_[jnt], q_target, t);

        if (sim_flag_) {
            double pos_actual = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
            double vel_actual = get_state(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
            double tau = sim_kp_ * (q - pos_actual) + sim_kd_ * (0.0 - vel_actual);
            set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, tau);
        } else {
            set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, q);
            set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
            set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
            set_command(jnt + "/" + hw_if::KP_SCALE, 1.0);
            set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
        }
    }

    // Lock wheels during transition
    if (has_wheels_) {
        for (const auto& jnt : wheel_joints_) {
            set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
            if (!sim_flag_) {
                set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, std::nan("1"));
                set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
                set_command(jnt + "/" + hw_if::KP_SCALE, 0.0);
                set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
            }
        }
    }

    // Transition complete
    if (t >= 1.0) {
        for (const auto& jnt : joints_) {
            const auto& cfg = joint_targets_[jnt];
            double q_target = (transition_target_ == TARGET_REST) ? cfg.q_rest : cfg.q_stand;
            leg_pos_cmd_[jnt] = q_target;
            leg_vel_cmd_[jnt] = 0.0;
            leg_eff_cmd_[jnt] = 0.0;
            leg_kp_cmd_[jnt] = (transition_target_ == TARGET_REST) ? 0.0 : 1.0;
            leg_kd_cmd_[jnt] = 1.0;
        }
        transition_completed_ = true;
        c_stt_ = ControllerState::ACTIVE;
        RCLCPP_INFO(
            get_node()->get_logger(), "Transition to %s complete, transitioning to ACTIVE",
            (transition_target_ == TARGET_REST) ? "rest" : "stand"
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Safety
// ═══════════════════════════════════════════════════════════════════════════

void OmniController::update_safety_monitoring()
{
    // Update temperature EMAs for all motor joints
    for (size_t i = 0; i < all_motor_joints_.size(); i++) {
        double temp = get_state(all_motor_joints_[i] + "/" + hw_if::TEMPERATURE);
        if (!ema_initialized_)
            temp_ema_[i] = temp;
        else
            temp_ema_[i] = ema_alpha_ * temp + (1.0 - ema_alpha_) * temp_ema_[i];
    }

    // Update voltage EMAs for all distributors
    for (size_t i = 0; i < distributor_names_.size(); i++) {
        double volt = get_state(distributor_names_[i] + "/" + std::string(hw_if::VOLTAGE));
        if (!ema_initialized_)
            volt_ema_[i] = volt;
        else
            volt_ema_[i] = ema_alpha_ * volt + (1.0 - ema_alpha_) * volt_ema_[i];
    }

    ema_initialized_ = true;

    // Wait for the EMA to warm up before evaluating transitions.
    // This avoids false triggers from noisy initial sensor readings.
    if (ema_warmup_counter_ < ema_window_samples_) {
        ema_warmup_counter_++;
        return;
    }

    evaluate_safety_transitions();
}

void OmniController::evaluate_safety_transitions()
{
    // Compute max temperature across all motors
    double max_temp = 0.0;
    std::string hottest_joint;
    for (size_t i = 0; i < temp_ema_.size(); i++) {
        if (temp_ema_[i] > max_temp) {
            max_temp = temp_ema_[i];
            hottest_joint = all_motor_joints_[i];
        }
    }

    // Compute min voltage across all distributors (skip if none)
    double min_volt = std::numeric_limits<double>::max();
    bool has_volt = !volt_ema_.empty();
    for (size_t i = 0; i < volt_ema_.size(); i++) {
        if (volt_ema_[i] < min_volt)
            min_volt = volt_ema_[i];
    }

    // Check NUC heartbeat (only after first heartbeat received, and if timeout > 0)
    bool heartbeat_lost = (heartbeat_timeout_ > 0.0) && heartbeat_received_ &&
                          (get_node()->now() - last_heartbeat_time_).seconds() > heartbeat_timeout_;

    // Pre-compute condition flags for critical triggers
    bool over_temp = max_temp > temp_critical_threshold_;
    bool under_volt = has_volt && min_volt < battery_min_voltage_;

    // Build reason string for SAFETY_CRITICAL log
    auto build_reason = [&]() -> std::string {
        std::string reason;
        if (heartbeat_lost)
            reason += "NUC heartbeat lost";
        if (over_temp) {
            if (!reason.empty())
                reason += ", ";
            reason += "motor over-temperature";
        }
        if (under_volt) {
            if (!reason.empty())
                reason += ", ";
            reason += "battery under-voltage";
        }
        return reason;
    };

    switch (safety_state_) {
    case SafetyState::SAFETY_NORMAL:
        if (over_temp || under_volt || heartbeat_lost) {
            safety_state_ = SafetyState::SAFETY_CRITICAL;
            RCLCPP_ERROR(
                get_node()->get_logger(), "SAFETY CRITICAL [%s]: temp=%.1f°C (%s), volt=%.1fV",
                build_reason().c_str(), max_temp, hottest_joint.c_str(), has_volt ? min_volt : 0.0
            );
        } else if (max_temp > temp_warning_threshold_) {
            safety_state_ = SafetyState::SAFETY_WARNING;
            RCLCPP_WARN(
                get_node()->get_logger(), "SAFETY WARNING: temp=%.1f°C (%s)", max_temp,
                hottest_joint.c_str()
            );
        }
        break;

    case SafetyState::SAFETY_WARNING:
        if (over_temp || under_volt || heartbeat_lost) {
            safety_state_ = SafetyState::SAFETY_CRITICAL;
            RCLCPP_ERROR(
                get_node()->get_logger(), "SAFETY CRITICAL [%s]: temp=%.1f°C (%s), volt=%.1fV",
                build_reason().c_str(), max_temp, hottest_joint.c_str(), has_volt ? min_volt : 0.0
            );
        } else if (max_temp < temp_warning_threshold_ - temp_recovery_hysteresis_) {
            safety_state_ = SafetyState::SAFETY_NORMAL;
            RCLCPP_INFO(get_node()->get_logger(), "Safety recovered from WARNING to NORMAL");
        } else {
            // Throttled warning at ~1Hz (500 ticks at 500Hz)
            if (++warn_throttle_counter_ >= 500) {
                warn_throttle_counter_ = 0;
                RCLCPP_WARN(
                    get_node()->get_logger(), "SAFETY WARNING: temp=%.1f°C (%s)", max_temp,
                    hottest_joint.c_str()
                );
            }
        }
        break;

    case SafetyState::SAFETY_STOPPED: {
        // Check recovery conditions
        bool temp_ok = max_temp < (temp_warning_threshold_ - temp_recovery_hysteresis_);
        bool volt_ok = !has_volt || (min_volt > battery_min_voltage_ + volt_recovery_hysteresis_);
        bool heartbeat_ok =
            !heartbeat_received_ || (heartbeat_timeout_ <= 0.0) ||
            (get_node()->now() - last_heartbeat_time_).seconds() <= heartbeat_timeout_;
        if (temp_ok && volt_ok && heartbeat_ok) {
            safety_state_ = SafetyState::SAFETY_NORMAL;
            // c_stt_ stays INACTIVE — user must call activate_srv
            RCLCPP_INFO(
                get_node()->get_logger(),
                "Safety recovered from STOPPED to NORMAL (activate_srv required to resume)"
            );
        }
        break;
    }

    default:
        // CRITICAL and DAMPING transitions are handled in update()
        break;
    }
}

void OmniController::update_damping(const rclcpp::Time& time)
{
    if (!damping_time_initialized_) {
        damping_start_time_ = time;
        damping_time_initialized_ = true;
        // Record current positions for all leg joints
        for (const auto& jnt : joints_)
            damping_q_start_[jnt] = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
    }

    double elapsed = (time - damping_start_time_).seconds();
    double t = std::clamp(elapsed / damping_duration_, 0.0, 1.0);

    // Lock wheels during damping
    if (has_wheels_) {
        for (const auto& jnt : wheel_joints_) {
            set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
            if (!sim_flag_) {
                set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, std::nan("1"));
                set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
                set_command(jnt + "/" + hw_if::KP_SCALE, 0.0);
                set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
            }
        }
    }

    if (critical_strategy_ == "damping") {
        // Hold current position, ramp kp_scale from 1.0 → 0.0 via cosine
        double kp_scale = cosine_interp(1.0, 0.0, t);
        for (const auto& jnt : joints_) {
            if (sim_flag_) {
                double pos_actual = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
                double vel_actual = get_state(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
                double tau = (sim_kp_ * kp_scale) * (damping_q_start_[jnt] - pos_actual)
                           + sim_kd_ * (0.0 - vel_actual);
                set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, tau);
            } else {
                set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, damping_q_start_[jnt]);
                set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
                set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
                set_command(jnt + "/" + hw_if::KP_SCALE, kp_scale);
                set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
            }
        }
    } else {
        // "default_config": interpolate to safe positions, kp stays at 1.0
        for (const auto& jnt : joints_) {
            double target_q = 0.0;
            try {
                target_q =
                    get_node()->get_parameter("safety.default_config." + jnt + ".q").as_double();
            } catch (...) {
                // Fallback to q_rest
                if (joint_targets_.count(jnt))
                    target_q = joint_targets_[jnt].q_rest;
            }
            double q = cosine_interp(damping_q_start_[jnt], target_q, t);
            if (sim_flag_) {
                double pos_actual = get_state(jnt + "/" + hardware_interface::HW_IF_POSITION);
                double vel_actual = get_state(jnt + "/" + hardware_interface::HW_IF_VELOCITY);
                double tau = sim_kp_ * (q - pos_actual) + sim_kd_ * (0.0 - vel_actual);
                set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, tau);
            } else {
                set_command(jnt + "/" + hardware_interface::HW_IF_POSITION, q);
                set_command(jnt + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
                set_command(jnt + "/" + hardware_interface::HW_IF_EFFORT, 0.0);
                set_command(jnt + "/" + hw_if::KP_SCALE, 1.0);
                set_command(jnt + "/" + hw_if::KD_SCALE, 1.0);
            }
        }
    }

    // Check if damping sequence is complete
    if (t >= 1.0) {
        safety_state_ = SafetyState::SAFETY_STOPPED;
        RCLCPP_WARN(get_node()->get_logger(), "Damping complete, transitioning to STOPPED");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════════

double OmniController::get_state(const std::string& key) const
{
    auto it = stt_idx_.find(key);
    if (it != stt_idx_.end())
        return state_interfaces_[it->second].get_value();
    return 0.0;
}

double OmniController::get_command(const std::string& key) const
{
    auto it = cmd_idx_.find(key);
    if (it != cmd_idx_.end())
        return command_interfaces_[it->second].get_value();
    return 0.0;
}

void OmniController::set_command(const std::string& key, double value)
{
    auto it = cmd_idx_.find(key);
    if (it != cmd_idx_.end())
        command_interfaces_[it->second].set_value(value);
}

} // namespace omni_controller

PLUGINLIB_EXPORT_CLASS(omni_controller::OmniController, controller_interface::ControllerInterface)
