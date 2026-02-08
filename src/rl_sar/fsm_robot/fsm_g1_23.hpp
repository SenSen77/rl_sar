/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef G1_23_FSM_HPP
#define G1_23_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"

// NOTE:
// This file defines an independent FSM for the 23DoF G1 variant (`robot_name=g1_23`).
// It is intentionally separated from `fsm_g1.hpp` (29DoF) so that:
// - config paths stay under `policy/g1_23/*` and never collide with `policy/g1/*`
// - future sim2real changes for 23DoF don't impact 29DoF behavior

namespace g1_23_fsm
{

static inline std::vector<float> GetWaistAnglesSafe(const RL& rl, const RobotState<float>* state)
{
    std::vector<float> waist_angles(3, 0.0f);
    if (!state) return waist_angles;
    if (!rl.params.Has("waist_joint_indices")) return waist_angles;

    auto waist_sdk_indices = rl.params.Get<std::vector<int>>("waist_joint_indices");
    for (size_t i = 0; i < waist_angles.size() && i < waist_sdk_indices.size(); ++i)
    {
        int mapped = rl.InverseJointMapping(waist_sdk_indices[i]);
        if (mapped >= 0 && static_cast<size_t>(mapped) < state->motor_state.q.size())
        {
            waist_angles[i] = state->motor_state.q[mapped];
        }
    }
    return waist_angles;
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_getup = 0.0f;

    void Enter() override
    {
        percent_getup = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLRoboMimicLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
            {
                return "RLFSMStateRLWholeBodyTrackingMimic";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRLRoboMimicLocomotion : public RLFSMState
{
public:
    RLFSMStateRLRoboMimicLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLRoboMimicLocomotion") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;
        rl.config_name = "robomimic/locomotion";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
        {
            return "RLFSMStateRLWholeBodyTrackingMimic";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::LB_DPadDown)
        {
            return "RLFSMStateJointIndexTest";
        }
        return state_name_;
    }
};

// Joint index test:
// - Hold a safe standing posture (default_dof_pos)
// - Drive ONE selected joint with a small sinusoidal offset
// - Use keys to change joint index and amplitude
//
// Intended use: verify joint_mapping, joint directions, and encoder_bias calibration on real robot.
class RLFSMStateJointIndexTest : public RLFSMState
{
public:
    RLFSMStateJointIndexTest(RL *rl) : RLFSMState(*rl, "RLFSMStateJointIndexTest") {}

    int joint_idx = 0;
    float amp = 0.10f;   // rad
    float hz = 0.50f;    // Hz
    unsigned long long tick = 0;
    unsigned long long last_print_tick = 0;

    void Enter() override
    {
        tick = 0;
        last_print_tick = 0;
        joint_idx = 0;
        amp = 0.10f;
        hz = 0.50f;
        rl.episode_length_buf = 0;
        std::cout << std::endl << LOGGER::NOTE
                  << "[JointIndexTest] Num7:idx++ Num8:idx--  +/-:amp  Space:amp=0  Num0:GetUp  Num9:GetDown  P:Passive"
                  << std::endl;
    }

    void Run() override
    {
        const int n = rl.params.Get<int>("num_of_dofs");
        auto q_def = rl.params.Get<std::vector<float>>("default_dof_pos");
        auto kp = rl.params.Get<std::vector<float>>("fixed_kp");
        auto kd = rl.params.Get<std::vector<float>>("fixed_kd");

        // Bounds safety
        if (joint_idx < 0) joint_idx = 0;
        if (joint_idx >= n) joint_idx = n - 1;

        // Handle in-state key events (non-interactive, edge-trigger not required)
        if (rl.control.current_keyboard == Input::Keyboard::Num7 || rl.control.current_gamepad == Input::Gamepad::DPadRight)
        {
            joint_idx = std::min(joint_idx + 1, n - 1);
            rl.control.ClearInput();
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num8 || rl.control.current_gamepad == Input::Gamepad::DPadLeft)
        {
            joint_idx = std::max(joint_idx - 1, 0);
            rl.control.ClearInput();
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Space)
        {
            amp = 0.0f;
            rl.control.ClearInput();
        }

        // Continuous sinusoid command on the selected joint
        float t = float(tick) * rl.params.Get<float>("dt");
        float offset = amp * std::sin(2.0f * float(M_PI) * hz * t);

        for (int i = 0; i < n; ++i)
        {
            fsm_command->motor_command.q[i] = q_def[i];
            fsm_command->motor_command.dq[i] = 0.0f;
            fsm_command->motor_command.kp[i] = kp[i];
            fsm_command->motor_command.kd[i] = kd[i];
            fsm_command->motor_command.tau[i] = 0.0f;
        }
        fsm_command->motor_command.q[joint_idx] = q_def[joint_idx] + offset;

        // Periodic print
        if (tick - last_print_tick >= 50)
        {
            last_print_tick = tick;
            std::string jn = "";
            if (rl.params.Has("joint_names"))
            {
                auto names = rl.params.Get<std::vector<std::string>>("joint_names");
                if ((int)names.size() > joint_idx) jn = names[joint_idx];
            }
            std::cout << "\r\033[K" << std::flush
                      << LOGGER::INFO << "[JointIndexTest] idx=" << joint_idx
                      << " name=" << jn
                      << " amp=" << amp
                      << " hz=" << hz
                      << std::flush;
        }

        tick++;
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadUp)
        {
            return "RLFSMStateRLWholeBodyTrackingMimic";
        }
        return state_name_;
    }
};

class RLFSMStateRLWholeBodyTrackingMimic : public RLFSMState
{
public:
    RLFSMStateRLWholeBodyTrackingMimic(RL *rl) : RLFSMState(*rl, "RLFSMStateRLWholeBodyTrackingMimic") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;

        rl.config_name = "whole_body_tracking";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);

            rl.motion_loader.reset();
            rl.motion_length = 0.0f;
            if (rl.params.Has("motion_file"))
            {
                std::string motion_file_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + rl.params.Get<std::string>("motion_file");
                float fps = 1.0f / (rl.params.Get<float>("dt") * rl.params.Get<int>("decimation"));
                rl.motion_loader = std::make_unique<MotionLoader>(motion_file_path, fps);
                rl.motion_length = rl.motion_loader->GetDuration();
                rl.motion_loader->Reset(fsm_state->imu.quaternion, GetWaistAnglesSafe(rl, fsm_state));
                std::cout << LOGGER::INFO << "Motion duration: " << rl.motion_length << "s" << std::endl;
            }

            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

        if (rl.motion_loader && rl.motion_length > 0.0f)
        {
            float motion_time = rl.episode_length_buf * rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
            motion_time = std::fmin(motion_time, rl.motion_length);
            float percent = motion_time / rl.motion_length;
            LOGGER::PrintProgress(percent, rl.config_name);
            rl.motion_loader->Update(motion_time);
        }
        else
        {
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        }

        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num6 || rl.control.current_gamepad == Input::Gamepad::LB_DPadDown)
        {
            return "RLFSMStateJointIndexTest";
        }
        return state_name_;
    }
};

} // namespace g1_23_fsm

class G1_23FSMFactory : public FSMFactory
{
public:
    G1_23FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<g1_23_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<g1_23_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<g1_23_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLRoboMimicLocomotion")
            return std::make_shared<g1_23_fsm::RLFSMStateRLRoboMimicLocomotion>(rl);
        else if (state_name == "RLFSMStateRLWholeBodyTrackingMimic")
            return std::make_shared<g1_23_fsm::RLFSMStateRLWholeBodyTrackingMimic>(rl);
        else if (state_name == "RLFSMStateJointIndexTest")
            return std::make_shared<g1_23_fsm::RLFSMStateJointIndexTest>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "g1_23"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLRoboMimicLocomotion",
            "RLFSMStateRLWholeBodyTrackingMimic",
            "RLFSMStateJointIndexTest",
        };
    }
    std::string GetInitialState() const override { return initial_state_; }

private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(G1_23FSMFactory, "RLFSMStatePassive")

#endif // G1_23_FSM_HPP

