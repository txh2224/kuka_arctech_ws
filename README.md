# kuka_arctech_ws

面向 KUKA ArcTech（KSS 8.x / ArcTech Advanced 3.8）的 ROS 2 Humble / C++17 工作区。`kuka_arctech_adapter` 接收 A 组提交的 `AtomicInstruction` 作业序列，完成语义映射、KUKA 专用校验、黄金模板 `SRC/DAT` 实例化、静态检查与 SHA-256 清单生成，并通过可切换的 `IKrcTransport`（Mock / Manual / 未来 Verified）执行或导出程序包并回传逐步结果。

## 工作区结构

| 包 | 职责 |
| --- | --- |
| [`atomic_adapter_interfaces`](src/atomic_adapter_interfaces) | 定义 `AtomicInstruction`、`ExecuteBackendJob`（Action）、`BackendStatus`（Topic）和 `StopBackendJob`（Service） |
| [`kuka_arctech_adapter`](src/kuka_arctech_adapter) | B 组核心适配层：`AdapterNode` 组件、校验器、语义映射器、模板引擎、静态检查器、执行器与 KRC Transport |
| [`sim_A`](src/sim_A) | 模拟 A 组客户端（C++17 组件节点），从 `config/*.json` 读取作业样例向 Adapter 发单并打印反馈、Manifest 与结果 |

## 当前进度

- **路线 A（ROS 2 契约与 Mock 闭环）**：已完成
  - Action Server（`/adapter/kuka_arctech/execute_job`）、状态发布（`/adapter/kuka_arctech/status`）、停止服务（`/adapter/kuka_arctech/stop_job`）
  - 单作业互斥、停机确认与 `KA_STOP_FAILED` 槽位锁定/重试恢复
  - 7 类原子指令的语义映射与 IR 校验（含单位转换、范围检查、焊接时序校验、`seam_id` 1~65535 校验）
  - `MockKrcTransport` 与 `ProgramExecutor` 逐步执行、全量 N 条 `step_results[]` 回传
- **路线 B（黄金模板、静态检查与人工导出兜底）**：已完成
  - `TemplateEngine`：基于真实 KSS 8.x 导出格式（`dry_run_motion`）的白名单占位符实例化引擎
  - `StaticChecker`：程序名、`DEF/END`、`DEFDAT/ENDDAT`、零残留占位符、引弧/收弧成对、`dry_run` 防误引弧、`WDAT` 白名单、未验证模板安全门及 SHA-256 `artifact_manifest.json`
  - `ManualKrcTransport`：将 `.src` / `.dat` / `artifact_manifest.json` 落盘导出，返回 `WAITING_OPERATOR` 等待人工导入 KRC
- **待接入 / 后续项**
  - 在真实 KRC 示教器上补录包含 `ARCON` / `ARCOFF` / `WDAT` 的焊接黄金模板，替换 `straight_weld` 占位模板
  - `VerifiedKrcTransport`（真机部署、启停与过程读数采集）
  - 自动化单元测试（`KA-T01 ~ KA-T10`）

## 处理流水线

```text
ExecuteBackendJob Goal
  │
  ├─ 1. Goal 外层校验 (JobValidator::validate_goal)
  ├─ 2. 语义映射与单位转换 (SemanticMapper -> KukaArcTechJobIR)
  ├─ 3. KUKA IR 与时序校验 (JobValidator::validate_ir)
  ├─ 4. 黄金模板实例化 (TemplateEngine -> .src / .dat)
  ├─ 5. 静态检查 & SHA-256 清单 (StaticChecker -> artifact_manifest.json)
  └─ 6. 执行 / 导出 (ProgramExecutor + IKrcTransport: Mock / Manual)
```

## ROS 2 接口

| 类型 | 名称 | 接口定义 | 说明 |
| --- | --- | --- | --- |
| Action | `/adapter/kuka_arctech/execute_job` | `atomic_adapter_interfaces/action/ExecuteBackendJob` | 提交单条或多步原子指令作业，回传进度与 `result_json` / `artifact_manifest_json` |
| Topic | `/adapter/kuka_arctech/status` | `atomic_adapter_interfaces/msg/BackendStatus` | 定时发布后端连接状态、当前程序/焊接状态与故障码 |
| Service | `/adapter/kuka_arctech/stop_job` | `atomic_adapter_interfaces/srv/StopBackendJob` | 请求停止当前作业；若发生 `KA_STOP_FAILED` 可通过本服务重试停机以释放槽位 |

### 支持的原子指令

| `instruction_id` | `capability_id` | 说明 |
| --- | --- | --- |
| `ROB-010` | `robot.motion.move_joint` | 关节运动（rad -> KRL deg `E6AXIS` / `SPTP`） |
| `ROB-011` | `robot.motion.move_linear` | 笛卡尔直线运动（m/rad -> KRL mm/deg `E6POS` / `SLIN`） |
| `ROB-015` | `robot.motion.follow_path` | 多路点连续轨迹（`SLIN` 序列） |
| `WLD-003` | `welder.recipe.select` | 焊接配方选择（映射为 `WDAT` 白名单引用） |
| `WLD-015` | `welder.arc.start` | 起弧请求（含 `lease_ms` 与 PLC `seam_id` 握手） |
| `WLD-016` | `welder.arc.stop` | 收弧请求 |
| `WLD-024` | `welder.measure.get_process` | 只读查询焊接电流/电压过程量（纯测量作业走只读查询路径，不生成运动程序） |

## 构建

需要 ROS 2 Humble、支持 C++17 的编译器、CMake 3.12+ 与 `colcon`。在工作区根目录执行 [`build.sh`](build.sh)（默认启用 `--symlink-install`，修改 `config/*.json`、`templates/*` 或 `launch/*.py` 无需重新编译）：

```zsh
./build.sh
source install/setup.zsh
```

使用 Bash 时改为 `source install/setup.bash`。每个新终端都需要先加载工作区环境。

## 启动与联调

### 1. 启动 Adapter（推荐 Launch 方式）

**Mock 联调模式**（默认，走完整模板生成 + 静态检查 + 模拟执行）：

```zsh
ros2 launch kuka_arctech_adapter kuka_arctech_adapter.launch.py transport_mode:=mock
```

**Manual 人工导出模式**（校验与静态检查通过后将 `.src` / `.dat` / `artifact_manifest.json` 落盘，默认输出到当前目录的 `output/`）：

```zsh
ros2 launch kuka_arctech_adapter kuka_arctech_adapter.launch.py transport_mode:=manual
```

`kuka_arctech_adapter.launch.py` 支持的参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `node_name` | `kuka_arctech_adapter` | 节点名称 |
| `log_level` | `info` | 日志级别（`debug` / `info` / `warn` / `error` / `fatal`） |
| `transport_mode` | `mock` | 传输模式：`mock` 或 `manual`（`verified` 尚未接入，传入会拒绝启动） |
| `manual_export_dir` | `output` | `manual` 模式下的程序包导出根目录（相对当前工作目录或绝对路径） |

### 2. 使用 `sim_A` 发送测试任务

另开终端加载环境后运行：

```zsh
# 发送 8 步完整联调作业（默认 config/job_8steps_demo.json）
ros2 launch sim_A sim_a.launch.py

# 发送单步 ROB-010 作业
ros2 launch sim_A sim_a.launch.py job_file:=job_single_rob010.json

# 模拟运行中途取消（例如 0.5 秒后发送 Cancel）
ros2 launch sim_A sim_a.launch.py cancel_after_sec:=0.5
```

### 3. 查看状态与调用停止服务

```zsh
# 订阅后端状态
ros2 topic echo /adapter/kuka_arctech/status

# 主动请求停止当前作业
ros2 service call /adapter/kuka_arctech/stop_job \
  atomic_adapter_interfaces/srv/StopBackendJob \
  '{job_id: "job_mock_002", reason: "manual stop test"}'
```

### 4. 组件容器方式启动（可选）

```zsh
# 终端 1：启动组件容器
ros2 run rclcpp_components component_container

# 终端 2：加载 / 查看 / 卸载组件
ros2 component load /ComponentManager kuka_arctech_adapter AdapterNode
ros2 component list
ros2 component unload /ComponentManager 1
```

## 配置与黄金模板

以下文件在 `--symlink-install` 下修改后重启节点即生效，无需重编：

- [`src/kuka_arctech_adapter/config/validator.json`](src/kuka_arctech_adapter/config/validator.json)：运动速度/加速度、工作空间边界、`max_tool_id`、`max_base_id`、`max_seam_id`（65535）、`wdat_allowlist`、允许测量通道等。
- [`src/kuka_arctech_adapter/config/mapper.json`](src/kuka_arctech_adapter/config/mapper.json)：程序名前缀、默认 `seam_id`、`tool_id_map`、`frame_id_map`、`recipe_wdat_map`。
- [`src/sim_A/config/job_8steps_demo.json`](src/sim_A/config/job_8steps_demo.json) / [`job_single_rob010.json`](src/sim_A/config/job_single_rob010.json)：A 组模拟发单 JSON 样例。
- [`src/kuka_arctech_adapter/templates/kss8_arctech38/`](src/kuka_arctech_adapter/templates/kss8_arctech38)：
  - `dry_run_motion/`：基于真实 KSS 8.x 导出的纯运动/空跑模板（`verified_on_krc: true`）。
  - `straight_weld/`：实焊占位模板（`verified_on_krc: false`）。**静态检查安全门会拦截所有 `verified_on_krc: false` 模板在非 Mock 模式下导出或部署**；待示教器补录真实 ArcTech `ARCON`/`ARCOFF`/`WDAT` 黄金模板后直接替换该目录文件即可。

## 错误码

遵循《KUKA ArcTech Adapter 开发任务规格书》第 12 节，填入 `ExecuteBackendJob.Result.error_code`（成功时为空字符串 `""`）：

| 错误码 | 场景 |
| --- | --- |
| `KA_JOB_INVALID` | 作业缺少参数、指令顺序不合法，或无法映射到 KUKA 后端 |
| `KA_TEMPLATE_NOT_FOUND` | 找不到匹配目标版本或工艺的黄金模板 |
| `KA_TEMPLATE_RENDER_FAILED` | 模板实例化失败（如未声明占位符或格式错误） |
| `KA_STATIC_CHECK_FAILED` | 生成程序包未通过静态校验（或试图在非 Mock 模式使用未验证模板），阻止部署 |
| `KA_KRC_UNREACHABLE` | 无法连接或探测 KRC |
| `KA_DEPLOY_FAILED` | 程序包部署或导出失败，保留原始错误证据 |
| `KA_PROGRAM_START_FAILED` | 控制器选择或启动程序失败，保留原始错误证据 |
| `KA_SEAM_PERMISSION_TIMEOUT` | 等待 PLC 或上级系统焊缝许可握手超时 |
| `KA_ARCTECH_FAULT` | ArcTech 焊接故障，在 `evidence` 中保留控制器原始故障码和消息 |
| `KA_PROGRAM_TIMEOUT` | 程序运行超时，应请求停止 |
| `KA_STOP_FAILED` | 停止未获确认，锁定作业槽位并上报故障，需人工检查或通过 `stop_job` 重试确认 |
