#include "omni_controller/wheel_ik.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <map>

using namespace omni_controller;

// ─── MecanumIK Tests ────────────────────────────────────────────────────────

class MecanumIKTest: public ::testing::Test {
protected:
    void SetUp() override
    {
        ik = std::make_unique<MecanumIK>();
        WheelIKConfig config;
        config.wheel_rad = 0.05;
        config.driveshaft_x = 0.235;
        config.driveshaft_y = 0.188;
        config.mecanum_angle = 135.0;
        std::vector<std::string> names = {"LF", "LH", "RF", "RH"};
        ik->configure(config, names);
    }
    std::unique_ptr<MecanumIK> ik;
};

TEST_F(MecanumIKTest, NumWheels) { EXPECT_EQ(ik->num_wheels(), 4u); }

TEST_F(MecanumIKTest, ZeroInputGivesZeroOutput)
{
    auto vels = ik->inverse(0.0, 0.0, 0.0);
    ASSERT_EQ(vels.size(), 4u);
    for (auto v : vels)
        EXPECT_NEAR(v, 0.0, 1e-10);
}

TEST_F(MecanumIKTest, ForwardXOnly)
{
    double vx = 1.0;
    auto wheel_vels = ik->inverse(vx, 0.0, 0.0);
    ASSERT_EQ(wheel_vels.size(), 4u);

    // RF and RH should be positive, LF and LH negative (or vice versa based on sign convention)
    // But all should have the same magnitude
    double mag = std::abs(wheel_vels[0]);
    for (auto v : wheel_vels)
        EXPECT_NEAR(std::abs(v), mag, 1e-10);

    // LF (-) and LH (-), RF (+) and RH (+)
    EXPECT_LT(wheel_vels[0], 0.0); // LF: -1/wr * vx
    EXPECT_LT(wheel_vels[1], 0.0); // LH: -1/wr * vx
    EXPECT_GT(wheel_vels[2], 0.0); // RF: +1/wr * vx
    EXPECT_GT(wheel_vels[3], 0.0); // RH: +1/wr * vx
}

TEST_F(MecanumIKTest, IKFKRoundTrip)
{
    // Test that FK(IK(twist)) ≈ twist
    double vx = 0.5, vy = 0.3, omega = 0.2;
    auto wheel_vels = ik->inverse(vx, vy, omega);
    auto recovered = ik->forward(wheel_vels);

    EXPECT_NEAR(recovered[0], vx, 1e-10);
    EXPECT_NEAR(recovered[1], vy, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
}

TEST_F(MecanumIKTest, IKFKRoundTripPureRotation)
{
    double omega = 1.5;
    auto wheel_vels = ik->inverse(0.0, 0.0, omega);
    auto recovered = ik->forward(wheel_vels);

    EXPECT_NEAR(recovered[0], 0.0, 1e-10);
    EXPECT_NEAR(recovered[1], 0.0, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
}

TEST_F(MecanumIKTest, IKFKRoundTripLateralOnly)
{
    double vy = 0.8;
    auto wheel_vels = ik->inverse(0.0, vy, 0.0);
    auto recovered = ik->forward(wheel_vels);

    EXPECT_NEAR(recovered[0], 0.0, 1e-10);
    EXPECT_NEAR(recovered[1], vy, 1e-10);
    EXPECT_NEAR(recovered[2], 0.0, 1e-10);
}

TEST_F(MecanumIKTest, WrongWheelCountThrows)
{
    MecanumIK bad_ik;
    WheelIKConfig config;
    std::vector<std::string> names = {"A", "B"};
    EXPECT_THROW(bad_ik.configure(config, names), std::invalid_argument);
}

// ─── DifferentialIK Tests ───────────────────────────────────────────────────

class DifferentialIKTest: public ::testing::Test {
protected:
    void SetUp() override
    {
        ik = std::make_unique<DifferentialIK>();
        WheelIKConfig config;
        config.wheel_rad = 0.1;
        config.track_width = 0.5;
        std::vector<std::string> names = {"LEFT", "RIGHT"};
        ik->configure(config, names);
    }
    std::unique_ptr<DifferentialIK> ik;
};

TEST_F(DifferentialIKTest, NumWheels) { EXPECT_EQ(ik->num_wheels(), 2u); }

TEST_F(DifferentialIKTest, ZeroInputGivesZeroOutput)
{
    auto vels = ik->inverse(0.0, 0.0, 0.0);
    ASSERT_EQ(vels.size(), 2u);
    EXPECT_NEAR(vels[0], 0.0, 1e-10);
    EXPECT_NEAR(vels[1], 0.0, 1e-10);
}

TEST_F(DifferentialIKTest, ForwardOnly)
{
    double vx = 1.0;
    auto vels = ik->inverse(vx, 0.0, 0.0);
    // Both wheels same speed: vx / wheel_rad = 1.0 / 0.1 = 10
    EXPECT_NEAR(vels[0], 10.0, 1e-10);
    EXPECT_NEAR(vels[1], 10.0, 1e-10);
}

TEST_F(DifferentialIKTest, PureRotation)
{
    double omega = 2.0;
    auto vels = ik->inverse(0.0, 0.0, omega);
    // left = -omega * track/2 / r = -2.0 * 0.25 / 0.1 = -5.0
    // right = +omega * track/2 / r = +5.0
    EXPECT_NEAR(vels[0], -5.0, 1e-10);
    EXPECT_NEAR(vels[1], 5.0, 1e-10);
}

TEST_F(DifferentialIKTest, IKFKRoundTrip)
{
    double vx = 0.5, omega = 0.3;
    auto wheel_vels = ik->inverse(vx, 0.0, omega);
    auto recovered = ik->forward(wheel_vels);

    EXPECT_NEAR(recovered[0], vx, 1e-10);
    EXPECT_NEAR(recovered[1], 0.0, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
}

TEST_F(DifferentialIKTest, LateralMotionIgnored)
{
    // vy is ignored in differential drive
    auto vels_no_vy = ik->inverse(1.0, 0.0, 0.0);
    auto vels_with_vy = ik->inverse(1.0, 999.0, 0.0);
    EXPECT_NEAR(vels_no_vy[0], vels_with_vy[0], 1e-10);
    EXPECT_NEAR(vels_no_vy[1], vels_with_vy[1], 1e-10);
}

TEST_F(DifferentialIKTest, WrongWheelCountThrows)
{
    DifferentialIK bad_ik;
    WheelIKConfig config;
    std::vector<std::string> names = {"A", "B", "C"};
    EXPECT_THROW(bad_ik.configure(config, names), std::invalid_argument);
}

TEST_F(DifferentialIKTest, SingleWheelPerSideWorks)
{
    // DifferentialIK always takes exactly 2 logical wheels.
    // With multi-wheel-per-side, the controller passes one name per group.
    DifferentialIK fresh;
    WheelIKConfig config;
    config.wheel_rad = 0.1;
    config.track_width = 0.5;
    std::vector<std::string> names = {"left_front", "right_front"};
    EXPECT_NO_THROW(fresh.configure(config, names));

    auto vels = fresh.inverse(1.0, 0.0, 0.0);
    ASSERT_EQ(vels.size(), 2u);
    EXPECT_NEAR(vels[0], 10.0, 1e-10);
    EXPECT_NEAR(vels[1], 10.0, 1e-10);
}

TEST(DifferentialMultiWheelConcept, IKOutputFansOutToGroups)
{
    // Demonstrates the multi-wheel pattern:
    // IK computes 2 velocities, controller duplicates to N wheels per side.
    DifferentialIK ik;
    WheelIKConfig config;
    config.wheel_rad = 0.1;
    config.track_width = 0.5;
    // Controller passes one representative name per group
    ik.configure(config, {"LEFT", "RIGHT"});

    double vx = 0.5, omega = 1.0;
    auto vels = ik.inverse(vx, 0.0, omega);

    // Simulate 2 wheels per side: each gets the same velocity
    std::vector<std::vector<std::string>> groups = {
        {"left_front", "left_rear"}, {"right_front", "right_rear"}
    };

    // Fan out: each joint in group g gets vels[g]
    std::map<std::string, double> joint_cmds;
    for (size_t g = 0; g < groups.size(); g++)
        for (const auto& jnt : groups[g])
            joint_cmds[jnt] = vels[g];

    // All left wheels get the same velocity
    EXPECT_DOUBLE_EQ(joint_cmds["left_front"], joint_cmds["left_rear"]);
    // All right wheels get the same velocity
    EXPECT_DOUBLE_EQ(joint_cmds["right_front"], joint_cmds["right_rear"]);
    // Left != right (since omega != 0)
    EXPECT_NE(joint_cmds["left_front"], joint_cmds["right_front"]);

    // Forward kinematics: average each group, recover twist
    double avg_left = (joint_cmds["left_front"] + joint_cmds["left_rear"]) / 2.0;
    double avg_right = (joint_cmds["right_front"] + joint_cmds["right_rear"]) / 2.0;
    auto recovered = ik.forward({avg_left, avg_right});

    EXPECT_NEAR(recovered[0], vx, 1e-10);
    EXPECT_NEAR(recovered[2], omega, 1e-10);
}

// ─── Factory Tests ──────────────────────────────────────────────────────────

TEST(WheelIKFactory, CreateMecanum)
{
    auto ik = create_wheel_ik("mecanum");
    ASSERT_NE(ik, nullptr);
    EXPECT_EQ(ik->num_wheels(), 4u);
}

TEST(WheelIKFactory, CreateDifferential)
{
    auto ik = create_wheel_ik("differential");
    ASSERT_NE(ik, nullptr);
    EXPECT_EQ(ik->num_wheels(), 2u);
}

TEST(WheelIKFactory, CreateNoneReturnsNull)
{
    auto ik = create_wheel_ik("none");
    EXPECT_EQ(ik, nullptr);
}

TEST(WheelIKFactory, UnknownTypeThrows)
{
    EXPECT_THROW(create_wheel_ik("swerve"), std::invalid_argument);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
