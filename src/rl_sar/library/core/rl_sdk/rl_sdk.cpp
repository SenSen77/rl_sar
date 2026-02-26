/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"

void RL::StateController(const RobotState<float>* state, RobotCommand<float>* command)
{
    auto updateState = [&](std::shared_ptr<FSMState> statePtr)
    {
        if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(statePtr))
        {
            rl_fsm_state->fsm_state = state;
            rl_fsm_state->fsm_command = command;
        }
    };
    for (auto& pair : fsm.states_)
    {
        updateState(pair.second);
    }

    fsm.Run();

    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
    }
}

std::vector<float> RL::ComputeObservation()
{
    std::vector<std::vector<float>> obs_list;

    for (const std::string &observation : this->params.Get<std::vector<std::string>>("observations"))
    {
        // ============= Base Observations =============
        if (observation == "lin_vel")
        {
            obs_list.push_back(this->obs.lin_vel * this->params.Get<float>("lin_vel_scale"));
        }
        else if (observation == "ang_vel")
        {
            // In ROS1 Gazebo, the coordinate system for angular velocity is in the world coordinate system.
            // In ROS2 Gazebo, mujoco and real robot, the coordinate system for angular velocity is in the body coordinate system.
            if (this->ang_vel_axis == "body")
            {
                obs_list.push_back(this->obs.ang_vel * this->params.Get<float>("ang_vel_scale"));
            }
            else if (this->ang_vel_axis == "world")
            {
                obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.Get<float>("ang_vel_scale"));
            }
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(this->obs.commands * this->params.Get<std::vector<float>>("commands_scale"));
        }
        else if (observation == "dof_pos")
        {
            auto default_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
            std::vector<float> dof_pos_rel;

            // Optional mjlab-parity: encoder bias in joint position observations.
            // mjlab tracking uses joint_pos_rel(biased=True) == (q + encoder_bias) - default.
            if (this->params.Get<bool>("use_encoder_bias", false))
            {
                auto encoder_bias = this->params.Get<std::vector<float>>("encoder_bias");
                if (encoder_bias.size() == this->obs.dof_pos.size() && encoder_bias.size() == default_dof_pos.size())
                {
                    dof_pos_rel = (this->obs.dof_pos + encoder_bias) - default_dof_pos;
                }
                else
                {
                    // Fallback if config is missing/mismatched.
                    dof_pos_rel = this->obs.dof_pos - default_dof_pos;
                }
            }
            else
            {
                dof_pos_rel = this->obs.dof_pos - default_dof_pos;
            }
            for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
            {
                dof_pos_rel[i] = 0.0f;
            }
            obs_list.push_back(dof_pos_rel * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(this->obs.dof_vel * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "actions")
        {
            obs_list.push_back(this->obs.actions);
        }
        // ============= Other Observations =============
        else if (observation == "whole_body_tracking/motion_command")
        {
            std::vector<float> motion_cmd;
            if (this->motion_loader)
            {
                auto joint_pos_sdk = this->motion_loader->GetJointPos();
                auto joint_vel_sdk = this->motion_loader->GetJointVel();
                // IMPORTANT:
                // - `joint_mapping` is used by the sim/real interface to map training-DOF order
                //   <-> actuator/sensor order.
                // - Whole-body-tracking also needs a mapping from training DOFs -> motion CSV/SDK DOFs.
                //   Use `motion_joint_mapping` for that purpose to avoid corrupting actuator/sensor mapping.
                auto motion_joint_mapping = this->params.Has("motion_joint_mapping")
                                                ? this->params.Get<std::vector<int>>("motion_joint_mapping")
                                                : this->params.Get<std::vector<int>>("joint_mapping");
                std::vector<float> joint_pos_training(motion_joint_mapping.size());
                std::vector<float> joint_vel_training(motion_joint_mapping.size());
                for (size_t i = 0; i < motion_joint_mapping.size(); ++i)
                {
                    joint_pos_training[i] = joint_pos_sdk[motion_joint_mapping[i]];
                    joint_vel_training[i] = joint_vel_sdk[motion_joint_mapping[i]];
                }
                motion_cmd.insert(motion_cmd.end(), joint_pos_training.begin(), joint_pos_training.end());
                motion_cmd.insert(motion_cmd.end(), joint_vel_training.begin(), joint_vel_training.end());
            }
            else
            {
                motion_cmd.resize(this->params.Get<int>("num_of_dofs") * 2, 0.0f);
            }
            obs_list.push_back(motion_cmd);
        }
        else if (observation == "whole_body_tracking/motion_anchor_ori_b")
        {
            std::vector<float> anchor_ori(6, 0.0f);
            if (this->motion_loader)
            {
                auto motion_joint_mapping = this->params.Has("motion_joint_mapping")
                                                ? this->params.Get<std::vector<int>>("motion_joint_mapping")
                                                : this->params.Get<std::vector<int>>("joint_mapping");
                auto inverse_motion_mapping = [&](int sdk_idx) -> int {
                    for (size_t i = 0; i < motion_joint_mapping.size(); ++i)
                    {
                        if (motion_joint_mapping[i] == sdk_idx) return static_cast<int>(i);
                    }
                    return -1;
                };
                std::vector<float> waist_angles(3, 0.0f);
                if (this->params.Has("waist_joint_indices"))
                {
                    auto waist_sdk_indices = this->params.Get<std::vector<int>>("waist_joint_indices");
                    for (size_t i = 0; i < waist_angles.size() && i < waist_sdk_indices.size(); ++i)
                    {
                        int mapped = inverse_motion_mapping(waist_sdk_indices[i]);
                        if (mapped >= 0 && static_cast<size_t>(mapped) < this->obs.dof_pos.size())
                        {
                            waist_angles[i] = this->obs.dof_pos[mapped];
                        }
                    }
                }
                std::vector<float> robot_torso_quat_w = MotionLoader::ComputeTorsoQuat(this->obs.base_quat, waist_angles);
                std::vector<float> ref_torso_quat_w = this->motion_loader->GetAnchorQuat();
                std::vector<float> init_quat = this->motion_loader->GetInitQuat();
                std::vector<float> motion_anchor_quat_w = QuaternionMultiply(init_quat, ref_torso_quat_w);
                std::vector<float> robot_quat_inv = QuaternionConjugate(robot_torso_quat_w);
                std::vector<float> relative_quat = QuaternionMultiply(robot_quat_inv, motion_anchor_quat_w);
                std::vector<float> rot_matrix = QuaternionToRotationMatrix(relative_quat);
                anchor_ori = MatrixFirstTwoColumns(rot_matrix);
            }
            obs_list.push_back(anchor_ori);
        }
        else if (observation == "RoboMimic_Deploy/phase")
        {
            float motion_time = this->episode_length_buf * this->params.Get<float>("dt") * this->params.Get<int>("decimation");
            float count = motion_time;
            float phase = count / this->motion_length;
            std::vector<float> phase_vec = {phase};
            obs_list.push_back(phase_vec);
        }
    }

    this->obs_dims.clear();
    for (const auto& obs : obs_list)
    {
       this->obs_dims.push_back(obs.size());
    }

    std::vector<float> obs;
    for (const auto& obs_vec : obs_list)
    {
        obs.insert(obs.end(), obs_vec.begin(), obs_vec.end());
    }
    std::vector<float> clamped_obs = clamp(obs, -this->params.Get<float>("clip_obs"), this->params.Get<float>("clip_obs"));

    // Optional debug: print key observation stats periodically for sim2sim mismatch diagnosis.
    if (this->params.Get<bool>("debug_print_obs", false) && (this->motiontime % 50 == 0))
    {
        // Reconstruct a few high-signal pieces from the current observation struct (not the flattened vector).
        // Projected gravity (body frame)
        auto g_b = QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec);
        float dof_pos_rel_abs_max = 0.0f;
        auto def = this->params.Get<std::vector<float>>("default_dof_pos");
        for (size_t i = 0; i < std::min(def.size(), this->obs.dof_pos.size()); ++i)
        {
            dof_pos_rel_abs_max = std::max(dof_pos_rel_abs_max, std::abs(this->obs.dof_pos[i] - def[i]));
        }

        // Whole-body-tracking motion command debug (first few values + max abs)
        bool has_motion = (this->motion_loader != nullptr);
        float motion_pos_abs_max = 0.0f;
        float motion_vel_abs_max = 0.0f;
        std::vector<float> motion_pos_first, motion_vel_first;

        // Elbow debug (left/right) for constant bias diagnosis.
        struct ElbowDbg
        {
            float q = 0.0f;
            float q_def = 0.0f;
            float q_rel = 0.0f;
            float q_ref = 0.0f;
            float q_ref_abs = 0.0f;
            float err_rel = 0.0f;
            float action = 0.0f;
            float q_target = 0.0f;
            float err_to_ref = 0.0f;
        };
        ElbowDbg elbow_l, elbow_r;
        const int ELBOW_L_IDX = 16;
        const int ELBOW_R_IDX = 21;

        auto action_scale = this->params.Get<std::vector<float>>("action_scale");
        auto safe_get = [](const std::vector<float>& v, int idx, float fallback = 0.0f) -> float {
            if (idx < 0 || static_cast<size_t>(idx) >= v.size()) return fallback;
            return v[static_cast<size_t>(idx)];
        };
        auto fill_elbow = [&](ElbowDbg& out, int dof_idx, float q_ref)
        {
            out.q = safe_get(this->obs.dof_pos, dof_idx, 0.0f);
            out.q_def = safe_get(def, dof_idx, 0.0f);
            out.q_rel = out.q - out.q_def;
            out.q_ref = q_ref;
            out.q_ref_abs = out.q_ref + out.q_def;
            out.err_rel = out.q_rel - out.q_ref;
            out.action = safe_get(this->obs.actions, dof_idx, 0.0f);
            float s = safe_get(action_scale, dof_idx, 0.0f);
            out.q_target = out.q_def + out.action * s;
            out.err_to_ref = out.q - out.q_ref;
        };
        if (has_motion)
        {
            auto joint_pos_sdk = this->motion_loader->GetJointPos();
            auto joint_vel_sdk = this->motion_loader->GetJointVel();
            auto motion_joint_mapping = this->params.Has("motion_joint_mapping")
                                            ? this->params.Get<std::vector<int>>("motion_joint_mapping")
                                            : this->params.Get<std::vector<int>>("joint_mapping");
            const size_t n = std::min<size_t>(motion_joint_mapping.size(), this->params.Get<int>("num_of_dofs"));
            motion_pos_first.reserve(std::min<size_t>(n, 5));
            motion_vel_first.reserve(std::min<size_t>(n, 5));
            for (size_t i = 0; i < n; ++i)
            {
                int idx = motion_joint_mapping[i];
                float p = (idx >= 0 && static_cast<size_t>(idx) < joint_pos_sdk.size()) ? joint_pos_sdk[idx] : 0.0f;
                float v = (idx >= 0 && static_cast<size_t>(idx) < joint_vel_sdk.size()) ? joint_vel_sdk[idx] : 0.0f;
                motion_pos_abs_max = std::max(motion_pos_abs_max, std::abs(p));
                motion_vel_abs_max = std::max(motion_vel_abs_max, std::abs(v));
                if (i < 5)
                {
                    motion_pos_first.push_back(p);
                    motion_vel_first.push_back(v);
                }
            }

            // Elbow ref angles from motion via mapping (training DOF idx -> motion CSV idx).
            int elbow_l_sdk = (motion_joint_mapping.size() > (size_t)ELBOW_L_IDX) ? motion_joint_mapping[ELBOW_L_IDX] : -1;
            int elbow_r_sdk = (motion_joint_mapping.size() > (size_t)ELBOW_R_IDX) ? motion_joint_mapping[ELBOW_R_IDX] : -1;
            float elbow_l_ref = (elbow_l_sdk >= 0 && static_cast<size_t>(elbow_l_sdk) < joint_pos_sdk.size()) ? joint_pos_sdk[elbow_l_sdk] : 0.0f;
            float elbow_r_ref = (elbow_r_sdk >= 0 && static_cast<size_t>(elbow_r_sdk) < joint_pos_sdk.size()) ? joint_pos_sdk[elbow_r_sdk] : 0.0f;
            fill_elbow(elbow_l, ELBOW_L_IDX, elbow_l_ref);
            fill_elbow(elbow_r, ELBOW_R_IDX, elbow_r_ref);
        }
        else
        {
            fill_elbow(elbow_l, ELBOW_L_IDX, 0.0f);
            fill_elbow(elbow_r, ELBOW_R_IDX, 0.0f);
        }

        auto print_vec = [](const std::vector<float>& v) {
            std::ostringstream oss;
            oss << "[";
            for (size_t i = 0; i < v.size(); ++i)
            {
                if (i) oss << ",";
                oss << v[i];
            }
            oss << "]";
            return oss.str();
        };

        // Last action debug (policy output after optional clamping)
        float action_abs_max = 0.0f;
        std::vector<float> action_first;
        for (size_t i = 0; i < this->obs.actions.size(); ++i)
        {
            action_abs_max = std::max(action_abs_max, std::abs(this->obs.actions[i]));
            if (i < 5) action_first.push_back(this->obs.actions[i]);
        }
        std::cout << std::endl
                  << LOGGER::INFO << "[obs] cmd=(" << this->obs.commands[0] << "," << this->obs.commands[1] << "," << this->obs.commands[2] << ")"
                  << " ang=(" << this->obs.ang_vel[0] << "," << this->obs.ang_vel[1] << "," << this->obs.ang_vel[2] << ")"
                  << " g_b=(" << g_b[0] << "," << g_b[1] << "," << g_b[2] << ")"
                  << " dof_rel_abs_max=" << dof_pos_rel_abs_max
                  << " action_abs_max=" << action_abs_max
                  << " action_first=" << print_vec(action_first)
                  << " has_motion=" << (has_motion ? 1 : 0)
                  << " motion_pos_abs_max=" << motion_pos_abs_max
                  << " motion_vel_abs_max=" << motion_vel_abs_max
                  << " motion_pos_first=" << print_vec(motion_pos_first)
                  << " motion_vel_first=" << print_vec(motion_vel_first)
                  << " elbowL(q=" << elbow_l.q << ",def=" << elbow_l.q_def << ",rel=" << elbow_l.q_rel
                  << ",ref=" << elbow_l.q_ref << ",ref_abs=" << elbow_l.q_ref_abs << ",err_rel=" << elbow_l.err_rel
                  << ",act=" << elbow_l.action << ",tgt=" << elbow_l.q_target
                  << ",err=" << elbow_l.err_to_ref << ")"
                  << " elbowR(q=" << elbow_r.q << ",def=" << elbow_r.q_def << ",rel=" << elbow_r.q_rel
                  << ",ref=" << elbow_r.q_ref << ",ref_abs=" << elbow_r.q_ref_abs << ",err_rel=" << elbow_r.err_rel
                  << ",act=" << elbow_r.action << ",tgt=" << elbow_r.q_target
                  << ",err=" << elbow_r.err_to_ref << ")"
                  << std::endl;
    }
    return clamped_obs;
}

void RL::InitObservations()
{
    this->obs.lin_vel = {0.0f, 0.0f, 0.0f};
    this->obs.ang_vel = {0.0f, 0.0f, 0.0f};
    this->obs.gravity_vec = {0.0f, 0.0f, -1.0f};
    this->obs.commands = {0.0f, 0.0f, 0.0f};
    // Quaternion convention in this project is [w, x, y, z] (scalar-first).
    // Identity quaternion is [1, 0, 0, 0].
    this->obs.base_quat = {1.0f, 0.0f, 0.0f, 0.0f};
    this->obs.dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->obs.dof_vel.clear();
    this->obs.dof_vel.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.actions.clear();
    this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->ComputeObservation();
}

void RL::InitOutputs()
{
    int num_of_dofs = this->params.Get<int>("num_of_dofs");
    this->output_dof_tau.clear();
    this->output_dof_tau.resize(num_of_dofs, 0.0f);
    this->output_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->output_dof_vel.clear();
    this->output_dof_vel.resize(num_of_dofs, 0.0f);

    // initialize last valid policy outputs with a safe standing target
    this->last_policy_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->last_policy_dof_vel.assign(num_of_dofs, 0.0f);
    this->last_policy_valid = false;
}

void RL::InitControl()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
}

void RL::InitJointNum(size_t num_joints)
{
    this->robot_state.motor_state.resize(num_joints);
    this->start_state.motor_state.resize(num_joints);
    this->now_state.motor_state.resize(num_joints);
    this->robot_command.motor_command.resize(num_joints);
}

void RL::InitRL(std::string robot_config_path)
{
    std::lock_guard<std::mutex> lock(this->model_mutex);

    this->ReadYaml(robot_config_path, "config.yaml");

    // init joint num first
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));

    // init rl
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();

    // init obs history
    const auto& observations_history = this->params.Get<std::vector<int>>("observations_history");  // avoid dangling reference
    if (!observations_history.empty())
    {
        int history_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;
        this->history_obs_buf = ObservationBuffer(1, this->obs_dims, history_length, this->params.Get<std::string>("observations_history_priority"));
    }

    // init model
    std::string model_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + this->params.Get<std::string>("model_name");
    this->model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!this->model)
    {
        throw std::runtime_error("Failed to load model from: " + model_path);
    }
}

void RL::ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau)
{
    // Keep behavior consistent with training: clip is handled at the policy output stage (see Forward()).
    std::vector<float> actions_scaled = actions * this->params.Get<std::vector<float>>("action_scale");
    std::vector<float> pos_actions_scaled = actions_scaled;
    std::vector<float> vel_actions_scaled(actions.size(), 0.0f);
    for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
    {
        pos_actions_scaled[i] = 0.0f;
        vel_actions_scaled[i] = actions_scaled[i];
    }
    auto default_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    auto use_encoder_bias = this->params.Get<bool>("use_encoder_bias", false);
    auto encoder_bias = this->params.Get<std::vector<float>>("encoder_bias");

    std::vector<float> all_actions_scaled = pos_actions_scaled + vel_actions_scaled;

    // Position target: q_target = default + action_scale * action  (mjlab uses default offset too)
    // Optional mjlab-parity: subtract encoder bias from commanded target so that
    // the effective target in the biased joint space matches training.
    output_dof_pos = pos_actions_scaled + default_dof_pos;
    if (use_encoder_bias && encoder_bias.size() == output_dof_pos.size())
    {
        output_dof_pos = output_dof_pos - encoder_bias;
    }
    output_dof_vel = vel_actions_scaled;

    // Torque target: tau = kp*(q_target - q) - kd*dq
    // Apply the same encoder_bias correction to q_target if enabled.
    std::vector<float> q_target_minus_q = all_actions_scaled + default_dof_pos - this->obs.dof_pos;
    if (use_encoder_bias && encoder_bias.size() == q_target_minus_q.size())
    {
        q_target_minus_q = q_target_minus_q - encoder_bias;
    }
    output_dof_tau = this->params.Get<std::vector<float>>("rl_kp") * q_target_minus_q - this->params.Get<std::vector<float>>("rl_kd") * this->obs.dof_vel;
    output_dof_tau = clamp(output_dof_tau, -this->params.Get<std::vector<float>>("torque_limits"), this->params.Get<std::vector<float>>("torque_limits"));
}

int RL::InverseJointMapping(int idx) const
{
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (size_t i = 0; i < joint_mapping.size(); ++i) {
        if (joint_mapping[i] == idx) return (int)i;
    }
    return -1;
}

void RL::TorqueProtect(const std::vector<float>& origin_output_dof_tau)
{
    std::vector<int> out_of_range_indices;
    std::vector<float> out_of_range_values;
    for (size_t i = 0; i < origin_output_dof_tau.size(); ++i)
    {
        float torque_value = origin_output_dof_tau[i];
        float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[i];
        float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[i];

        if (torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if (!out_of_range_indices.empty())
    {
        for (size_t i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            float value = out_of_range_values[i];
            float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[index];
            float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[index];

            std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
    }
}

void RL::AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold)
{
    // Use QuaternionToEuler from vector_math.hpp
    std::vector<float> euler = QuaternionToEuler(quaternion);
    float roll = euler[0] * 57.2958f;   // Convert to degrees
    float pitch = euler[1] * 57.2958f;

    if (std::fabs(roll) > roll_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Roll exceeds " << roll_threshold << " degrees. Current: " << roll << " degrees." << std::endl;
    }
    if (std::fabs(pitch) > pitch_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Pitch exceeds " << pitch_threshold << " degrees. Current: " << pitch << " degrees." << std::endl;
    }
}

#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

static int kbhit()
{
    static bool initialized = false;
    static termios original_term;

    // Initialize terminal to non-canonical mode on first call
    if (!initialized)
    {
        tcgetattr(STDIN_FILENO, &original_term);

        termios new_term = original_term;
        new_term.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
        new_term.c_cc[VMIN] = 0;   // Non-blocking read
        new_term.c_cc[VTIME] = 0;  // No timeout

        tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

        // Register cleanup function to restore terminal on exit
        static bool cleanup_registered = false;
        if (!cleanup_registered)
        {
            std::atexit([]() {
                tcsetattr(STDIN_FILENO, TCSANOW, &original_term);
            });
            cleanup_registered = true;
        }

        initialized = true;
    }

    // Non-blocking read of a single character
    char c;
    int result = read(STDIN_FILENO, &c, 1);

    return (result == 1) ? (unsigned char)c : -1;
}

void RL::KeyboardInterface()
{
    int c = kbhit();
    if (c > 0)
    {
        switch (c)
        {
        case '0': this->control.SetKeyboard(Input::Keyboard::Num0); break;
        case '1': this->control.SetKeyboard(Input::Keyboard::Num1); break;
        case '2': this->control.SetKeyboard(Input::Keyboard::Num2); break;
        case '3': this->control.SetKeyboard(Input::Keyboard::Num3); break;
        case '4': this->control.SetKeyboard(Input::Keyboard::Num4); break;
        case '5': this->control.SetKeyboard(Input::Keyboard::Num5); break;
        case '6': this->control.SetKeyboard(Input::Keyboard::Num6); break;
        case '7': this->control.SetKeyboard(Input::Keyboard::Num7); break;
        case '8': this->control.SetKeyboard(Input::Keyboard::Num8); break;
        case '9': this->control.SetKeyboard(Input::Keyboard::Num9); break;
        case 'a': case 'A': this->control.SetKeyboard(Input::Keyboard::A); break;
        case 'b': case 'B': this->control.SetKeyboard(Input::Keyboard::B); break;
        case 'c': case 'C': this->control.SetKeyboard(Input::Keyboard::C); break;
        case 'd': case 'D': this->control.SetKeyboard(Input::Keyboard::D); break;
        case 'e': case 'E': this->control.SetKeyboard(Input::Keyboard::E); break;
        case 'f': case 'F': this->control.SetKeyboard(Input::Keyboard::F); break;
        case 'g': case 'G': this->control.SetKeyboard(Input::Keyboard::G); break;
        case 'h': case 'H': this->control.SetKeyboard(Input::Keyboard::H); break;
        case 'i': case 'I': this->control.SetKeyboard(Input::Keyboard::I); break;
        case 'j': case 'J': this->control.SetKeyboard(Input::Keyboard::J); break;
        case 'k': case 'K': this->control.SetKeyboard(Input::Keyboard::K); break;
        case 'l': case 'L': this->control.SetKeyboard(Input::Keyboard::L); break;
        case 'm': case 'M': this->control.SetKeyboard(Input::Keyboard::M); break;
        case 'n': case 'N': this->control.SetKeyboard(Input::Keyboard::N); break;
        case 'o': case 'O': this->control.SetKeyboard(Input::Keyboard::O); break;
        case 'p': case 'P': this->control.SetKeyboard(Input::Keyboard::P); break;
        case 'q': case 'Q': this->control.SetKeyboard(Input::Keyboard::Q); break;
        case 'r': case 'R': this->control.SetKeyboard(Input::Keyboard::R); break;
        case 's': case 'S': this->control.SetKeyboard(Input::Keyboard::S); break;
        case 't': case 'T': this->control.SetKeyboard(Input::Keyboard::T); break;
        case 'u': case 'U': this->control.SetKeyboard(Input::Keyboard::U); break;
        case 'v': case 'V': this->control.SetKeyboard(Input::Keyboard::V); break;
        case 'w': case 'W': this->control.SetKeyboard(Input::Keyboard::W); break;
        case 'x': case 'X': this->control.SetKeyboard(Input::Keyboard::X); break;
        case 'y': case 'Y': this->control.SetKeyboard(Input::Keyboard::Y); break;
        case 'z': case 'Z': this->control.SetKeyboard(Input::Keyboard::Z); break;
        case ' ': this->control.SetKeyboard(Input::Keyboard::Space); break;
        case '\n': case '\r': this->control.SetKeyboard(Input::Keyboard::Enter); break;
        case 27:  // Escape sequence (for arrow keys on Unix/Linux/macOS)
        {
            char seq[2];
            // Try to read escape sequence non-blockingly
            if (read(STDIN_FILENO, &seq[0], 1) == 1)
            {
                if (seq[0] == '[')
                {
                    if (read(STDIN_FILENO, &seq[1], 1) == 1)
                    {
                        switch (seq[1])
                        {
                        case 'A': this->control.SetKeyboard(Input::Keyboard::Up); break;
                        case 'B': this->control.SetKeyboard(Input::Keyboard::Down); break;
                        case 'C': this->control.SetKeyboard(Input::Keyboard::Right); break;
                        case 'D': this->control.SetKeyboard(Input::Keyboard::Left); break;
                        default: break;
                        }
                    }
                }
                else
                {
                    // Plain escape key
                    this->control.SetKeyboard(Input::Keyboard::Escape);
                }
            }
            else
            {
                // Plain escape key
                this->control.SetKeyboard(Input::Keyboard::Escape);
            }
        } break;
        default:  break;
        }
    }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node)
{
    std::vector<T> values;
    for (const auto &val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

void RL::ReadYaml(const std::string& file_path, const std::string& file_name)
{
    std::string config_path = std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[file_path];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    for (auto it = config.begin(); it != config.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        this->params.config_node[key] = it->second;
    }
}

void RL::CSVInit(std::string robot_path)
{
    csv_filename = std::string(POLICY_DIR) + "/" + robot_path + "/motor";

    // Uncomment these lines if need timestamp for file name
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_cal_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_est_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_target_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_vel_" << i << ","; }

    file << std::endl;

    file.close();
}

void RL::CSVLogger(const std::vector<float>& torque, const std::vector<float>& tau_est, const std::vector<float>& joint_pos, const std::vector<float>& joint_pos_target, const std::vector<float>& joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << torque[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << tau_est[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos_target[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_vel[i] << ","; }

    file << std::endl;

    file.close();
}

bool RLFSMState::Interpolate(
    float& percent,
    const std::vector<float>& start_pos,
    const std::vector<float>& target_pos,
    float duration_seconds,
    const std::string& description,
    bool use_fixed_gains)
{
    if (percent >= 1.0f)
    {
        return false;
    }

    if (percent == 0.0f)
    {
        float max_diff = 0.0f;
        for (size_t i = 0; i < start_pos.size() && i < target_pos.size(); ++i)
        {
            max_diff = std::max(max_diff, std::abs(start_pos[i] - target_pos[i]));
        }

        if (max_diff < 0.1f)
        {
            percent = 1.0f;
        }
    }

    int required_frames = std::max(1, static_cast<int>(std::ceil(duration_seconds / rl.params.Get<float>("dt"))));
    float step = 1.0f / required_frames;

    percent += step;
    percent = std::min(percent, 1.0f);

    auto kp = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kp") : rl.params.Get<std::vector<float>>("rl_kp");
    auto kd = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kd") : rl.params.Get<std::vector<float>>("rl_kd");

    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
    {
        fsm_command->motor_command.q[i] = (1 - percent) * start_pos[i] + percent * target_pos[i];
        fsm_command->motor_command.dq[i] = 0;
        fsm_command->motor_command.kp[i] = kp[i];
        fsm_command->motor_command.kd[i] = kd[i];
        fsm_command->motor_command.tau[i] = 0;
    }

    if (!description.empty())
    {
        LOGGER::PrintProgress(percent, description);
    }

    if (percent >= 1.0f)
    {
        return false;
    }

    return true;
}

void RLFSMState::RLControl()
{
    // NOTE:
    // The policy thread pushes pos/vel (and tau) into separate queues.
    // A naive `try_pop(pos) && try_pop(vel)` can drop `pos` if `vel` isn't available yet,
    // leading to tiny/unstable motions (or inheriting previous state's kp=0).
    // We instead drain queues independently and always command the last known good outputs.

    std::vector<float> tmp;
    while (rl.output_dof_pos_queue.try_pop(tmp))
    {
        if (!tmp.empty())
        {
            rl.last_policy_dof_pos = std::move(tmp);
            rl.last_policy_valid = true;
        }
    }
    while (rl.output_dof_vel_queue.try_pop(tmp))
    {
        if (!tmp.empty())
        {
            rl.last_policy_dof_vel = std::move(tmp);
        }
    }

    const int n = rl.params.Get<int>("num_of_dofs");
    // If we don't have any policy outputs yet, hold current posture with RL gains.
    if (!rl.last_policy_valid && fsm_state && (int)fsm_state->motor_state.q.size() >= n)
        {
        rl.last_policy_dof_pos = fsm_state->motor_state.q;
        rl.last_policy_dof_vel.assign(n, 0.0f);
        rl.last_policy_valid = true;
    }

    auto kp = rl.params.Get<std::vector<float>>("rl_kp");
    auto kd = rl.params.Get<std::vector<float>>("rl_kd");
    for (int i = 0; i < n; ++i)
            {
        if ((int)rl.last_policy_dof_pos.size() >= n) fsm_command->motor_command.q[i] = rl.last_policy_dof_pos[i];
        if ((int)rl.last_policy_dof_vel.size() >= n) fsm_command->motor_command.dq[i] = rl.last_policy_dof_vel[i];
        fsm_command->motor_command.kp[i] = kp[i];
        fsm_command->motor_command.kd[i] = kd[i];
            fsm_command->motor_command.tau[i] = 0;
    }
}
