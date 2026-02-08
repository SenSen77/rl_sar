# G1 23DoF 室内行走（瓷砖/木板）RL 计划与进度

更新时间：2026-01-26

## 目标（你当前最重要的指标）

- **场景**：室内地面（瓷砖 + 木板），小范围摩擦变化
- **机器人**：Unitree G1 **23DoF**
- **要求**：**稳当**、**噪音小**、**动作平滑**（不追求极限速度）
- **链路**：`robot_lab` 训练/挑选 checkpoint → 导出策略 → `rl_sar` Gazebo **sim2sim**

## 当前状态（已完成）

### 1) 已定位训练日志与关键结论（robot_lab）

训练日志：
- `robot_lab/logs/rsl_rl/unitree_g1_rough/2025-12-24_20-00-24/`
- TensorBoard 事件文件：`events.out.tfevents...`

关键现象（来自曲线统计）：
- **已学会稳定存活**：回合长度接近上限，`time_out` 占比约 0.96。
- **最优 checkpoint 不在最后**：`Train/mean_reward` 峰值在 **iter=8461 (~41.58)**，最后 **iter=9999 (~35.94)**。
- **后期回落更像“课程更难了”**：`Curriculum/terrain_levels` 从 ~5.11 涨到 ~5.35，同时回合长度从 ~995 降到 ~978。
- **抖动/激烈动作的信号**：分项 reward 里 `joint_pos_penalty`、`action_rate_l2`、`joint_pos_limits`、`feet_slide` 后期更吃亏；同时 `mean_noise_std` 较高（~1.82）。

### 2) 已确认算法/关键超参（PPO）

来自 `.../params/agent.yaml`：
- PPO：`clip_param=0.2`
- **entropy_coef=0.008**
- `schedule=adaptive` + `desired_kl=0.01`
- `learning_rate=1e-3`
- `num_steps_per_env=24`（配合 4096 env）

直觉解释（给新手）：
- **entropy_coef** 越大，越鼓励“随机探索”，容易带来动作噪声与抖动。
- **adaptive KL** 会根据 KL 偏离程度调学习率，有利于稳定，但在强随机化/高难度 curriculum 下，后期表现可能被“更难分布”拉低。

### 3) 已确认 sim2sim 接口对齐（rl_sar）

`rl_sar/policy/g1_23/robomimic/locomotion/config.yaml` 明确写了：
- 观测顺序与训练对齐：`["ang_vel","gravity_vec","commands","dof_pos","dof_vel","actions"]`
- `num_of_dofs=23`，`num_observations=78`
- `action_scale=0.25`（与 RobotLab `JointPositionAction(scale=0.25)` 一致）
- actions clip：当前也是 **±100**（与训练侧一致）

控制频率（rl_sar）：
- `dt=0.005`，`decimation=4` → **RL 推理 50Hz**（`loop_rl = dt * decimation`）

## 最新进度（2026-01-26）

- **MuJoCo**：`g1_23` 已能 **较稳定行走**，但 **步子偏小**（可通过命令范围/训练微调改进）。
- **Gazebo**：仍然 **不够稳定**（更像 Gazebo 物理/控制器/摩擦差异导致的 sim2sim gap，需要单独对齐）。

## 踩坑记录（高频问题 → 解决方案）

### 1) “关节几乎不动 / 2 秒直着倒”，但 IsaacLab play 正常

**现象**：MuJoCo/Gazebo 都出现动作幅度很小，进入 locomotion 很快直着倒；而 IsaacLab play 步态正常。  
**结论**：优先怀疑 **部署链路**（而不是 policy 本身）。

- **坑 A：策略输出队列丢帧导致“几乎不更新”**
  - **原因**：`RLControl()` 以前用 `try_pop(pos) && try_pop(vel)`；policy 线程分别 push pos/vel，容易出现 “pos pop 成功但 vel 尚未入队 → pos 被丢掉”
  - **修复**：改为独立 drain 队列并缓存 `last_policy_*`，始终使用最后有效输出
  - **文件**：`rl_sar/src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`、`rl_sdk.hpp`

- **坑 B：动作裁剪过紧把步态“掐死”**
  - **原因**：如果把动作 clip 设到 `[-1, 1]`，可能会把策略输出压扁，导致抬腿/迈步不足
  - **处理**：调试阶段保持与训练一致的 **±100** 裁剪（真正约束动作应在训练里通过 reward/正则完成）
  - **文件**：`rl_sar/policy/g1_23/robomimic/locomotion/config.yaml`

- **坑 C：四元数约定不一致（潜在）**
  - **约定**：本项目四元数是 **[w, x, y, z]**，单位四元数应为 **[1, 0, 0, 0]**
  - **文件**：`rl_sar/src/rl_sar/library/core/rl_sdk/rl_sdk.cpp`

- **诊断工具：观测打印开关**
  - **用途**：快速判断 `g_b`（投影重力）、关节相对姿态幅度、命令是否正确
  - **开关**：在 `config.yaml` 里加 `debug_print_obs: true`

### 2) “开始再训练”但直接报错退出（配置命名不一致）

**现象**：启动 `RobotLab-Isaac-Velocity-IndoorFlat-Unitree-G1-v0` 训练时，报：
`AttributeError: 'RewardsCfg' object has no attribute 'feet_air_time_biped'`

**原因**：当前版本 reward 字段名是 `feet_air_time`，而不是 `feet_air_time_biped`。

**修复**：已将 `UnitreeG1FlatEnvCfg` 中对应字段改为 `self.rewards.feet_air_time`。

### 3) “换 task/config 后无法 resume”：load_run 不能填路径

**现象**：`--resume --load_run xxx --checkpoint yyy.pt` 在切换到新 task（新 `experiment_name`）后报：
`FileNotFoundError: ... /logs/rsl_rl/<experiment_name>`

**原因**：
- `train.py` 的 resume 逻辑会在 `logs/rsl_rl/{experiment_name}/` 下按 run 名称正则查找
- `--load_run` 期望的是 **run 文件夹名/正则**（例如 `2025-12-24_20-00-24`），不是路径
- 如果 `experiment_name` 变了，就会在新目录下找不到旧 run

**推荐做法**（二选一）：
- 方案A（最简单）：让 `experiment_name` 指回旧实验根目录，然后从旧 run resume，但用 `--run_name` 区分新实验
- 方案B：把旧 run 目录复制/软链接到新 `logs/rsl_rl/{experiment_name}/` 下，再 resume

### 4) “换成 IndoorFlat 后 resume 报 critic size mismatch”

**现象**（典型报错）：
`size mismatch for critic.0.weight: ... checkpoint is [512, 296], current is [512, 109]`

**原因**：
- PPO checkpoint 保存的是 **actor+critic**
- 换 task/config 后，**critic 的观测维度变了**（例如从 296 → 109）
- 因此无法严格加载 critic 权重

**解决方案**：
- 推荐：**actor-only resume**（加载 actor + std/log_std，critic 重新初始化），用于跨环境微调
- 命令见下方「再训练命令（actor-only resume）」。

### 5) play.py 加载 checkpoint 报 critic mismatch（跨 task 播放）

**现象**：用 `RobotLab-Isaac-Velocity-Rough-Unitree-G1-v0` 播放一个 IndoorFlat 训练出的 checkpoint 时，`ppo_runner.load()` 可能报 critic 维度不一致。

**原因**：play 默认严格加载 actor+critic；而不同 task/config 的 critic 观测维度可能不同（例如 rough 有 height_scan → 296）。

**解决**：播放时也使用 **actor-only**：

```bash
python3 scripts/reinforcement_learning/rsl_rl/play.py \
  --task RobotLab-Isaac-Velocity-Rough-Unitree-G1-v0 \
  --checkpoint /ABS/PATH/TO/model_xxxx.pt \
  --resume_actor_only \
  --num_envs 1
```

## 下一步行动计划（按“最小改动、可验证”排序）

### 0. Gazebo 只能站 2 秒：先做“部署侧止血”

现象：Gazebo sim2sim 只能站 2 秒左右。

高概率原因（按优先级）：
- **动作裁剪过宽**：策略输出是“无量纲动作”，会乘 `action_scale=0.25` 再加到默认站姿。若动作被允许到 ±100，会造成关节目标瞬间跳变。

已做的止血改动（本仓库）：
- 已将 `rl_sar/policy/g1_23/robomimic/locomotion/config.yaml` 的 `clip_actions_*` 从 ±100 收紧到 **[-1, 1]**。

预期：
- 如果之前是“动作突跳”导致摔倒，收紧裁剪后 **站立时间应明显变长**（至少能稳定进入走路阶段）。

### A. 立刻提升 sim2sim 体验：先用最优 checkpoint 播放/导出

目的：避免拿到“最后回落的策略”，直接从最优段开始做 sim2sim。

推荐先试（robot_lab log 目录里已有）：
- `model_8450.pt` 或 `model_8500.pt`

播放命令（带 checkpoint）示例：

```bash
python3 /home/Mercy/Documents/1.Project/robot_lab/scripts/reinforcement_learning/rsl_rl/play.py \
  --task RobotLab-Isaac-Velocity-Rough-Unitree-G1-v0 \
  --checkpoint /home/Mercy/Documents/1.Project/robot_lab/logs/rsl_rl/unitree_g1_rough/2025-12-24_20-00-24/model_8450.pt \
  --num_envs 1
```

导出策略（若需要）：
- 该 run 已有：`.../exported/policy.pt`、`.../exported/policy.onnx`
- 如果你之后导出的是别的 checkpoint，原则是：**导出时要锁定到同一个 checkpoint**（不要默认导出最后的）。

### B. 针对“室内稳、静、平滑”的训练改进（建议 A/B 小实验）

你的目标不是越快越好，而是 **更安静更平滑**。最有效的旋钮通常是“减少随机动作 + 增强平滑惩罚”。

#### 实验 B1（首推）：降低探索强度（减少噪声/抖动）

改动小、收益大：
- 将 `entropy_coef` 从 `0.008` 降到 `0.001 ~ 0.003`

预期：
- `Loss/entropy` 末段更容易下降/不再上升
- `Policy/mean_noise_std` 下降或稳定
- `action_rate_l2`、`joint_pos_penalty` 变好（更平滑、关节更少“打摆子”）

#### 实验 B2：增强动作平滑惩罚（让“抖动换 reward”不再划算）

训练侧 reward 权重（当前）：
- `action_rate_l2: weight=-0.005`
- `joint_pos_penalty: weight=-1.0`
- `feet_slide: weight=-0.2`

做法：
- 逐步增大 `action_rate_l2` 的惩罚（例如 2x～4x 作为对照）
- 或者对 `joint_acc_l2` / `joint_vel_l2`（如启用）做小幅惩罚

预期：
- 走路更“柔”、落脚更稳、噪音更小
- 可能牺牲一点 tracking 峰值，但更符合室内需求

#### 实验 B3（可选）：把训练随机化收敛到“瓷砖/木板”

你现在域随机化很强（摩擦、质量、惯量、COM、增益、推）。室内 sim2sim 如果目标是“瓷砖/木板”，可以先把随机化范围收缩到合理区间，再逐步放开。

建议先做“室内版本”的随机化：
- 摩擦系数范围更窄（瓷砖/木板一般都偏滑但变化不大）
- 推/外力先降低或关掉（室内走路先稳，再抗扰）

### C. sim2sim 落地（rl_sar）

你现在 `rl_sar` 的推理链路是：
- 50Hz 读取状态 → 组 obs（含 last_action）→ `model->forward()` → 输出 joint pos targets（经 `ComputeOutput`）→ PD 控制

为了“更平滑更安静”，在不改训练的情况下，也可以做两类“部署侧”优化（可选）：
- **动作低通滤波**：对 `output_dof_pos` 做轻微一阶滤波（注意不要引入大相位滞后）
- **命令限幅与斜坡**：对 `/cmd_vel` 做 ramp（防止瞬时指令导致大冲击）

> 注意：部署侧滤波是“补救”，训练侧平滑奖励才是“根治”。最佳路径是两者结合：训练先学平滑，部署再做轻微滤波。

## 进度追踪（下一个里程碑）

- [ ] 用 `model_8450.pt` / `model_8500.pt` 在 RobotLab play 验证“更稳更顺”
- [ ] 将选定 checkpoint 的 `policy.pt` 放入 `rl_sar/policy/g1_23/robomimic/locomotion/`（或更新 `model_name` 指向新文件）
- [ ] Gazebo sim2sim：在瓷砖/木板（不同摩擦）下跑 5–10 分钟，记录：
  - 跌倒率/保护触发次数（如有）
  - 关节目标变化幅度（抖动程度）
  - 足端滑移（若能估计）
- [ ] 训练侧做 B1（entropy_coef 降低）A/B 对照，比较：
  - `mean_reward`、`mean_episode_length`
  - `Loss/entropy`、`Policy/mean_noise_std`
  - `Episode_Reward/action_rate_l2`、`joint_pos_penalty`、`feet_slide`

## 再训练（室内平地微调）——已新增 task

为室内瓷砖/木板 sim2sim，我新增了一个更贴近室内分布的训练任务：
- **task**：`RobotLab-Isaac-Velocity-IndoorFlat-Unitree-G1-v0`
- **特点**：
  - 平地（plane），无地形 curriculum
  - 命令范围更小（更像室内走路）
  - 摩擦随机化收窄到“瓷砖/木板”合理区间
  - 关闭周期推力（push）
  - 加强平滑惩罚（`action_rate_l2` 等）

从你原 run 继续训练（fine-tune）示例命令（在 `robot_lab/` 下运行）：

```bash
python3 scripts/reinforcement_learning/rsl_rl/train.py \
  --task RobotLab-Isaac-Velocity-IndoorFlat-Unitree-G1-v0 \
  --resume --load_run logs/rsl_rl/unitree_g1_rough/2025-12-24_20-00-24 --checkpoint model_8500.pt \
  --run_name indoor_flat_ft \
  --num_envs 4096 \
  --headless
```

## 下一步（Gazebo 稳定性专项）

因为 MuJoCo 已稳、Gazebo 仍不稳，建议把 Gazebo gap 当成单独任务对齐：
- **地面摩擦**：瓷砖/木板的 `mu`/`mu2`（以及接触模型）与 IsaacLab/MuJoCo 的等效摩擦差异
- **关节控制器**：Gazebo 控制器是否真正使用 `kp/kd`，以及更新频率是否与 50Hz 对齐
- **IMU/角速度坐标系**：ROS1/ROS2 与 Gazebo 插件输出坐标系差异（`ang_vel_axis`）



