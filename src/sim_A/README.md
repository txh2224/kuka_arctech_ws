# sim_A

模拟 A 组（Runtime / 上位编排层）的 ROS 2 Humble / C++17 测试客户端包，基于 `rclcpp_components` 实现。节点从 `config/` 下的 JSON 文件读取 `ExecuteBackendJob` 作业配置，向 B 组的 `kuka_arctech_adapter` 发送 Action Goal，并在终端彩色打印实时 `Feedback`、`artifact_manifest_json` 与完整 `result_json`。

## 目录结构

```text
sim_A/
├── config/                         # 预设单步 / 多步测试作业 JSON（修改无需重编）
├── include/sim_a_node.h            # SimANode 组件声明
├── src/sim_a_node.cpp              # Action Client 加载、发单、取消与结果打印实现
├── launch/sim_a.launch.py          # 启动脚本与参数声明
├── CMakeLists.txt
└── package.xml
```

## 快速启动

先在一个终端启动 `kuka_arctech_adapter`（`mock` 或 `manual` 模式）：

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch kuka_arctech_adapter kuka_arctech_adapter.launch.py transport_mode:=mock
```

再开一个终端启动 `sim_A` 发送作业：

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh

# 默认运行 8 步空跑焊接演示（job_8steps_demo.json）
ros2 launch sim_A sim_a.launch.py

# 通过 job_file 指定预设任务（支持 config/ 下文件名，或任意相对/绝对路径）
ros2 launch sim_A sim_a.launch.py job_file:=job_multi_motion_square.json
```

> 说明：工作区通过 `./build.sh`（`--symlink-install`）构建后，直接修改现有 `config/*.json` 重跑即生效；若新增了新的 `.json` 文件，跑一次 `./build.sh` 更新符号链接即可通过文件名直接引用。

## 预设作业一览（`config/`）

### 1. 单条指令预设

| 文件名 | `job_id` | 指令类型 | 说明 |
| --- | --- | --- | --- |
| `job_single_rob010.json` | `job_single_rob-010` | `ROB-010` (`MoveJoint`) | 单步 6 轴关节运动（生成 `SPTP`） |
| `job_single_rob011.json` | `job_single_rob011` | `ROB-011` (`MoveLinear`) | 单步笛卡尔直线运动至 `[0.52, 0.14, 0.38]`（生成 `SLIN`） |
| `job_single_rob015.json` | `job_single_rob015` | `ROB-015` (`FollowCartesianPath`) | 单步 4 路点连续折线轨迹跟随（生成 4 段 `SLIN`） |
| `job_single_wld024.json` | `job_single_wld024` | `WLD-024` (`GetWeldMeasurements`) | 纯只读查询焊接电流/电压（走只读查询分支，不生成 `.src/.dat`） |

### 2. 多条指令连续动作预设

以下多步预设均为 `dry_run: true`，既可在 `mock` 模式下完整走通进度与结果回传，也可在 `transport_mode:=manual` 下直接导出经过静态检查的 KRL `.src` / `.dat` 空跑程序包（默认导出至 `output/AI_<JOB_ID>/`）：

| 文件名 | `job_id` | 步数 | 动作序列说明 |
| --- | --- | --- | --- |
| `job_8steps_demo.json` | `job_mock_002` | 8 步 | 标准 8 步空跑焊接联调样例：`ROB-010` 接近 -> `ROB-011` 到起弧点 -> `WLD-003` 选配方 -> `WLD-015` 起弧 -> `ROB-015` 焊缝轨迹 -> `WLD-024` 过程采样 -> `WLD-016` 收弧 -> `ROB-011` 退刀 |
| `job_multi_motion_square.json` | `job_motion_square` | 4 步 | **矩形巡航**：`ROB-010` 关节就位 -> `ROB-011` 下探起点 -> `ROB-015` 走 100×100 mm 闭环矩形（5 路点） -> `ROB-011` 抬刀退回 |
| `job_multi_joint_sweep.json` | `job_joint_sweep` | 3 步 | **关节往返摆动**：3 条 `ROB-010`（`A1..A6` 命名）以不同角速度/角加速度连续往返并回原位 |
| `job_multi_2seams_dryrun.json` | `job_2seams_dryrun` | 7 步 | **双段轨迹空跑**：`ROB-010` 就位 -> `WLD-003` 选配方 -> `ROB-015` 第 1 段轨迹 -> `ROB-011` 抬刀跨段过渡 -> `ROB-015` 第 2 段轨迹 -> `WLD-024` 过程采样 -> `ROB-011` 退刀 |

## 常用测试命令示例

```zsh
# 1. 单步直线运动 / 折线轨迹 / 只读采样
ros2 launch sim_A sim_a.launch.py job_file:=job_single_rob011.json
ros2 launch sim_A sim_a.launch.py job_file:=job_single_rob015.json
ros2 launch sim_A sim_a.launch.py job_file:=job_single_wld024.json

# 2. 多步矩形巡航 / 关节往返 / 双段轨迹空跑
ros2 launch sim_A sim_a.launch.py job_file:=job_multi_motion_square.json
ros2 launch sim_A sim_a.launch.py job_file:=job_multi_joint_sweep.json
ros2 launch sim_A sim_a.launch.py job_file:=job_multi_2seams_dryrun.json

# 3. 测试中途取消（发送 Goal 后 0.3 秒自动触发 Action Cancel）
ros2 launch sim_A sim_a.launch.py job_file:=job_multi_motion_square.json cancel_after_sec:=0.3
```

## Launch 参数说明

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `job_file` | `job_8steps_demo.json` | 作业 JSON 文件名（优先在 `sim_A/config/` 下查找）或直接传入文件路径 |
| `action_name` | `/adapter/kuka_arctech/execute_job` | 目标 Action Server 名称 |
| `wait_server_sec` | `5.0` | 等待 Action Server 上线的超时时间（秒） |
| `cancel_after_sec` | `0.0` | 若 `> 0`，在 Goal 被接收后延迟指定秒数自动发送取消请求 |
| `auto_shutdown` | `true` | 收到最终结果（或失败/超时）后自动退出 `sim_a_node` |
