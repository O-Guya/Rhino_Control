#include <gtest/gtest.h>
#include "dm_gripper_core/gripper_state_machine.hpp"
#include "dm_gripper_core/impedance_controller.hpp"

using namespace dm_gripper_core;
using namespace dm_motor_driver;

static GripperKinematics make_kin()
{
    return GripperKinematics(7.957747e-4);
}

static ContactDetector::Params make_contact_params()
{
    return { .tau_threshold = 2.0f, .pos_stuck_threshold = 0.001f, .window_size = 5 };
}

static MotorState make_state(float pos = 0.0f, float vel = 0.0f, float tau = 0.0f)
{
    MotorState s;
    s.pos   = pos;
    s.vel   = vel;
    s.tau   = tau;
    s.valid = true;
    return s;
}

TEST(StateMachine, InitialStateIsIdle)
{
    GripperStateMachine sm(make_kin(), make_contact_params());
    EXPECT_EQ(sm.state(), GripperState::IDLE);
}

TEST(StateMachine, CommandWidthTransitionsToApproaching)
{
    GripperStateMachine sm(make_kin(), make_contact_params());
    sm.set_controller(std::make_unique<ImpedanceController>(
        ImpedanceController::Params{20.0f, 0.5f, 5.0f}));
    sm.command_width(0.05);
    EXPECT_EQ(sm.state(), GripperState::APPROACHING);
}

TEST(StateMachine, CommandOpenTransitionsToReleasing)
{
    GripperStateMachine sm(make_kin(), make_contact_params());
    sm.set_controller(std::make_unique<ImpedanceController>(
        ImpedanceController::Params{20.0f, 0.5f, 5.0f}));
    sm.command_open(0.10);
    EXPECT_EQ(sm.state(), GripperState::RELEASING);
}

TEST(StateMachine, ContactTransitionsToGrasping)
{
    GripperStateMachine sm(make_kin(), make_contact_params());
    sm.set_controller(std::make_unique<ImpedanceController>(
        ImpedanceController::Params{20.0f, 0.5f, 5.0f}));
    sm.command_width(0.02);

    // Inject stuck + high-torque states to trigger contact detection
    MotorState contact_state = make_state(1.0f, 0.0f, 3.0f);
    for (int i = 0; i < 20; i++) {
        sm.update(contact_state);
    }
    // After contact + CONTACTING cycle → should be GRASPING
    EXPECT_EQ(sm.state(), GripperState::GRASPING);
}

TEST(StateMachine, UpdateWithoutControllerDoesNotCrash)
{
    GripperStateMachine sm(make_kin(), make_contact_params());
    // No controller set
    MotorCommand cmd = sm.update(make_state());
    // Should return a safe hold command
    EXPECT_EQ(cmd.kp, 0.0f);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
