## mjlab → rl_sar（G1 23DOF WBT）踩坑记录

本文记录“mjlab 训练的 G1 23DOF whole_body_tracking 策略”接入 `rl_sar` 的 MuJoCo sim2sim 时，常见的“起飞/打滚/站不稳”等问题与修复方法。

### 1) 策略能在 mjlab `play` 正常，但在 rl_sar WBT 一切换就“起飞”
- **现象**：Passive/locomotion OK；切到 `whole_body_tracking` 立刻弹飞或爆力矩。
- **根因**：rl_sar 侧 PD 内环的 `rl_kp/rl_kd/torque_limits` 与 mjlab 训练时的 MuJoCo actuator 等效参数不一致。
  - rl_sar 会用：
    \[
    \\tau = K_p (q_{target} - q) + K_d (\\dot q_{target} - \\dot q)
    \]
    并再按 `torque_limits` clamp。
  - 如果 `torque_limits` 设成 300、Kp/Kd 偏大，策略输出稍微偏一点就会把机器人弹飞。
- **修复**：
  - 将 `policy/g1_23/whole_body_tracking/config.yaml` 的 `rl_kp/rl_kd` 改成 mjlab 等效 stiffness/damping。
  - 将 `policy/g1_23/base.yaml` 的 `torque_limits` 改成 mjlab 训练一致的 effort_limit（如 88/139/50/25 体系）。

### 2) “不再起飞，但在地上打滚/站不稳 1s 内摔”
这通常是 **观测不一致** 导致策略输出发散。

#### 2.1 动作裁剪范围过大（非常常见）
- **现象**：起飞修好后仍然乱动，动作幅度明显不对。
- **根因**：rl_sar 的策略输出裁剪是按 `clip_actions_lower/upper` 做的。若设成 ±100，会允许策略输出远大于训练时的范围，经过 `action_scale` 变成很大的 position target。
- **修复**：
  - 把 `policy/g1_23/whole_body_tracking/config.yaml` 的：
    - `clip_actions_lower/upper` 改成 **±1**（与 RSL-RL 典型训练一致）。

#### 2.2 Motion CSV 的 fps / torso 参考计算不一致
- **现象**：mjlab OK，但 rl_sar WBT 参考动作明显“错拍”，导致策略跟踪崩。
- **根因A（fps）**：G1 WBT 状态机里 `MotionLoader` 的 fps 取：
  - `fps = 1 / (dt * decimation)`，对 g1_23 默认是 **50Hz**。
  - 如果你给的是 **60Hz CSV**，MotionLoader 会按 50Hz 的 dt 去算 duration/速度，时间对不上。
- **修复A**：
  - 生成 **50Hz 重采样** 的 CSV，并在 `config.yaml` 中使用它（例如 `G1_Take_102.bvh_50hz_23dof.csv`）。

- **根因B（23DOF 下的 waist roll/pitch）**：
  - rl_sar 的 `MotionLoader` 在算 `motion_anchor_ori_b` 时，会硬编码把 motion joint[12,13,14] 当作 waist yaw/roll/pitch。
  - 但 23DOF 机器人只有 waist_yaw；roll/pitch 不应参与 torso 参考。
- **修复B**：
  - 对 CSV 的 joint[13], joint[14]（waist roll/pitch）置零，生成 `*_23dof.csv` 版本并使用。

### 3) `--registry-name your-org/motions/motion-name` 怎么用？和 DOF 有关吗？
- **结论**：有关。
  - mjlab 的 tracking 训练需要 `motion.npz`，其中 `joint_pos/joint_vel` 的维度必须与训练机器人 DOF 匹配（本项目为 23）。
  - 你可以：
    - **本地模式**：训练时直接用 `--env.commands.motion.motion-file /path/to/motion.npz`
    - **WandB Registry 模式**：先上传 motion artifact，再 `--registry-name entity/motions/name`
- **常见坑**：WandB entity 权限不足（400/403），导致 `csv_to_npz` 上传失败；这时先走本地 `motion.npz` 训练最稳。

### 4) 快速排查 checklist（推荐顺序）
1. `model_name` 指向正确的 `.pt`（TorchScript）文件
2. `clip_actions_lower/upper` 是否为 ±1
3. `action_scale` 与 mjlab 一致
4. `rl_kp/rl_kd/torque_limits` 与 mjlab actuator 等效一致
5. motion CSV 是否：
   - 50Hz（匹配 dt*decimation）
   - 23DOF 场景下 roll/pitch 已置零

