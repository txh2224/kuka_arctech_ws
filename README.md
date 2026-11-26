# kuka_arctech_ws

面向 KUKA ArcTech（KSS 8.x / ArcTech Advanced 3.8）的 ROS 2 Humble / C++17 工作区。`kuka_arctech_adapter`（B 组）接收上位系统（A 组）下发的 `AtomicInstruction` 原子指令序列，完成语义映射、KUKA 边界与工艺时序校验、黄金模板 `SRC/DAT` 实例化、静态安全门检查与 SHA-256 清单生成，并通过可插拔的 `IKrcTransport`（`Mock` / `Manual` / 预留 `Verified`）执行或导出程序包、回传全量逐步结果。

---

## 一、工作区目录结构

```text
kuka_arctech_ws/
├── build.sh                                    # 一键 colcon --symlink-install 构建脚本
├── src/
│   ├── atomic_adapter_interfaces/              # [契约层] A/B 组冻结 ROS 2 通信接口定义
│   │   ├── msg/AtomicInstruction.msg           #   单步原子指令消息
│   │   ├── msg/BackendStatus.msg               #   后端周期状态消息
│   │   ├── action/ExecuteBackendJob.action     #   作业执行 Action (Goal / Result / Feedback)
│   │   └── srv/StopBackendJob.srv              #   紧急/主动停止服务
│   │
│   ├── kuka_arctech_adapter/                   # [核心层] B 组 KUKA ArcTech 适配器包
│   │   ├── include/ & src/                     #   核心业务模块（按流水线职责解耦）
│   │   │   ├── adapter_node.*                  #     ROS 2 组件入口：单作业互斥槽位、状态广播、停机锁槽恢复
│   │   │   ├── job_validator.*                 #     双阶段校验器：Goal 外层校验 + IR 限位/时序/白名单校验
│   │   │   ├── semantic_mapper.*               #     语义映射器：AtomicInstruction JSON -> KukaArcTechJobIR
│   │   │   ├── job_ir.h / error_code.h         #     内部中间表示 (IR) 与 11 个标准 KA_* 错误码定义
│   │   │   ├── template_engine.*               #     模板引擎：白名单占位符渲染生成 KRL .src / .dat
│   │   │   ├── static_checker.*                #     静态检查器：语法/成对指令/未验证模板安全门 + Manifest 生成
│   │   │   ├── program_executor.*              #     作业执行编排器：只读分流、PLC 焊缝握手、超时/取消与结果汇总
│   │   │   ├── krc_transport.h                 #     控制器传输抽象接口 (IKrcTransport)
│   │   │   ├── mock_krc_transport.*            #     Mock 仿真后端（联调与故障注入单测）
│   │   │   └── manual_krc_transport.*          #     Manual 人工兜底后端（原子落盘导出至 output/）
│   │   │
│   │   ├── utils/                              #   [支撑层] 无状态通用工具与旁路诊断库
│   │   │   └── include/ & src/
│   │   │       ├── pose_conversion.*           #     ROS 米/四元数 <-> KUKA 毫米/ZYX 欧拉角互转
│   │   │       ├── result_json.*               #     统一组装契约要求的 result_json / step_results / evidence
│   │   │       ├── sha256.*                    #     生成 .src / .dat 的 SHA-256 指纹
│   │   │       ├── debug_recorder.*            #     异步后台写入 debug/*.jsonl 全链路通信审计日志
│   │   │       └── terminal_color.h            #     终端日志 ANSI 状态配色常量
│   │   │
│   │   ├── config/                             #   运行期外部配置（热修改无需重编）
│   │   │   ├── mapper.json                     #     程序名前缀、默认 seam_id、工具/基座/WDAT 映射表
│   │   │   └── validator.json                  #     关节/笛卡尔限位、最大速度加速度、WDAT 与测量通道白名单
│   │   ├── templates/kss8_arctech38/           #   KRL 黄金模板库
│   │   │   ├── dry_run_motion/                 #     已验证纯运动/空跑黄金模板 (verified_on_krc: true)
│   │   │   └── straight_weld/                  #     实焊占位模板 (verified_on_krc: false，非 Mock 模式拦截)
│   │   ├── launch/kuka_arctech_adapter.launch.py
│   │   └── test/test_kuka_arctech_adapter.cpp  #   自动化单元测试套件 (KA-T01 ~ KA-T09)
│   │
│   └── sim_A/                                  # [测试端] 模拟 A 组客户端包（详见 src/sim_A/README.md）
│       ├── config/*.json                       #   8 组单步/多步预设作业样例
│       ├── include/ & src/sim_a_node.*         #   SimANode Action Client 组件节点
│       └── launch/sim_a.launch.py
│
├── output/                                     # [运行期生成] Manual 模式导出的 .src/.dat/manifest (gitignored)
└── debug/                                      # [运行期生成] DebugRecorder 记录的 JSONL 通信日志 (gitignored)
```

---

## 二、系统架构与模块间关系

### 1. 模块协作拓扑图

```mermaid
flowchart TB
    subgraph GroupA["A 组上位编排层 / 测试客户端"]
        SimA["sim_A (SimANode)<br/>config/*.json"]
    end

    subgraph Contract["ROS 2 冻结通信契约 (atomic_adapter_interfaces)"]
        Act["Action: /adapter/kuka_arctech/execute_job<br/>(ExecuteBackendJob)"]
        Srv["Service: /adapter/kuka_arctech/stop_job<br/>(StopBackendJob)"]
        Top["Topic: /adapter/kuka_arctech/status<br/>(BackendStatus, 2Hz)"]
    end

    SimA -->|"发送 Goal / Cancel"| Act
    Act -->|"Feedback / Result"| SimA
    SimA -.->|"请求急停/重试解锁"| Srv
    Top -.->|"状态订阅"| SimA

    subgraph Adapter["B 组核心包 (kuka_arctech_adapter)"]
        Node["AdapterNode<br/>单作业槽位互斥 · 工作线程管理 · KA_STOP_FAILED 锁槽保护"]

        Act <--> Node
        Srv <--> Node
        Node --> Top

        subgraph Pipeline["编译与安全校验流水线"]
            ValGoal["1. JobValidator::validate_goal<br/>外层字段与空指令检查"]
            Mapper["2. SemanticMapper::map<br/>JSON 解析 · 单位/位姿转换 · 生成 KukaArcTechJobIR"]
            ValIR["3. JobValidator::validate_ir<br/>限位 · 速度 · 起收弧成对 · WDAT 白名单校验"]
            QueryGate{"纯只读查询?<br/>is_read_only_query_job<br/>(仅含 WLD-024)"}
            Tpl["4. TemplateEngine::render<br/>按黄金模板渲染 .src / .dat"]
            Check["5. StaticChecker::check<br/>KRL 语法/残留占位符/安全门检查<br/>计算 SHA-256 生成 artifact_manifest.json"]
        end

        Exec["6. ProgramExecutor::execute<br/>逐步推进 · PLC seam_id 握手 · 超时监控 · 停机收敛"]

        subgraph Transports["控制器传输抽象层 (IKrcTransport)"]
            ITrans["IKrcTransport (抽象接口)"]
            MockT["MockKrcTransport<br/>内存仿真执行 · 支持故障注入"]
            ManualT["ManualKrcTransport<br/>原子写入 output/AI_xxx/<br/>返回 WAITING_OPERATOR"]
            VerT["VerifiedKrcTransport<br/>(预留真机 KRC / PLC 通信)"]
        end

        subgraph Configs["配置与黄金模板资产"]
            CfgV["config/validator.json"]
            CfgM["config/mapper.json"]
            Tpls["templates/kss8_arctech38/*"]
        end

        subgraph Utils["通用支撑层 (utils/)"]
            UPose["pose_conversion<br/>米/四元数 ↔ 毫米/欧拉角"]
            UJson["result_json<br/>组装契约 JSON 结果"]
            USha["sha256<br/>文件指纹计算"]
            UDebug["DebugRecorder<br/>异步落盘 debug/*.jsonl"]
        end
    end

    Node --> ValGoal --> Mapper --> ValIR --> QueryGate
    QueryGate -->|"是（跳过模板与部署）"| Exec
    QueryGate -->|"否（运动 / 焊接作业）"| Tpl --> Check --> Exec

    Exec --> ITrans
    Node -.->|"周期读取连接状态 / 锁槽重试 stop"| ITrans
    ITrans <|.. MockT
    ITrans <|.. ManualT
    ITrans <|.. VerT

    CfgV -.-> ValGoal
    CfgV -.-> ValIR
    CfgV -.-> Check
    CfgM -.-> Mapper
    Tpls -.-> Tpl

    Mapper -.-> UPose
    UJson -.-> UPose
    Node -.-> UJson
    Exec -.-> UJson
    Check -.-> USha
    Node -.-> UDebug
```

### 2. 各模块职责与数据交接表

| 层级 | 模块 | 输入 | 输出 | 关键依赖 / 外部配置 |
| --- | --- | --- | --- | --- |
| **接入控制层** | `AdapterNode` | Action `Goal` / `Cancel`、`StopBackendJob` 请求 | Action `Feedback` / `Result`、500ms `BackendStatus` | 持有单槽位 `active_job`，串联后续全部组件；旁路调用 `utils::DebugRecorder` 与 `utils::make_rejected_result_json` |
| **编译流水线** | `JobValidator` | 原始 `Goal` 或 `KukaArcTechJobIR` | `ErrorCode`（通过为 `OK`，拒绝为 `KA_JOB_INVALID`） | 读取 [`validator.json`](src/kuka_arctech_adapter/config/validator.json) 限位、速度阈值、`seam_id` 上限与白名单 |
| **编译流水线** | `SemanticMapper` | `ExecuteBackendJob::Goal`（含各 `*_json` 字符串） | 结构化中间表示 `KukaArcTechJobIR` | 读取 [`mapper.json`](src/kuka_arctech_adapter/config/mapper.json)；调用 `utils::ros_to_kuka_pose` 完成坐标系与单位换算 |
| **编译流水线** | `TemplateEngine` | `KukaArcTechJobIR` | `ProgramPackage`（内存中的 `.src` 与 `.dat` 文本） | 读取 [`templates/kss8_arctech38/`](src/kuka_arctech_adapter/templates/kss8_arctech38) 下的 `template_manifest.json`、`golden.src`、`golden.dat` |
| **编译流水线** | `StaticChecker` | `ProgramPackage` + `KukaArcTechJobIR` + 当前传输模式 | 填充 `artifact_manifest_json` 或返回 `KA_STATIC_CHECK_FAILED` | 读取 [`validator.json`](src/kuka_arctech_adapter/config/validator.json) `wdat_allowlist`；调用 `utils::sha256_hex`；拦截非 Mock 模式下的未验证模板 |
| **执行与传输层** | `ProgramExecutor` | `KukaArcTechJobIR` + 可选 `ProgramPackage` + `stop_requested` 原子标志 | `ExecutionResult`（终态、错误码、完整 N 条 `step_results`） | 独占调用 `IKrcTransport` 的部署/启停/焊缝握手接口；调用 `utils::result_json` 构造逐步证据 |
| **执行与传输层** | `IKrcTransport` | 部署包、启停命令、`seam_id` 许可请求 | `TransportResult` / `SeamPermissionResult` | `MockKrcTransport`（仿真执行）与 `ManualKrcTransport`（`.tmp` + `rename` 原子导出至 `output/`） |
| **支撑工具层** | `utils/*` | 位姿结构体、字节流、执行步骤、ROS 通信事件 | 欧拉角/四元数、SHA-256 字符串、JSON 文本、`debug/*.jsonl` | 纯算法或独立后台日志队列，不反向耦合上层业务状态机 |

### 3. 关键并发与安全机制

1. **单作业互斥与停机锁槽保护**：`AdapterNode` 同一时刻只允许一个 `active_job` 运行。若取消或超时停机时底层 `stop_program()` 未确认成功（包括断连 `KA_KRC_UNREACHABLE`），统一收敛为 `KA_STOP_FAILED` 并**锁定作业槽位**、持续广播故障码，直到运维或上位通过 `/adapter/kuka_arctech/stop_job` 重试停机确认成功后才释放槽位接收新任务。
2. **双路径分流**：纯 `WLD-024`（`welder.measure.get_process`）作业由 `is_read_only_query_job()` 识别，直接跳过黄金模板渲染与 KRL 静态检查，避免无意义的运动文件生成。
3. **未验证模板硬安全门**：实焊模板 `straight_weld` 当前标记为 `verified_on_krc: false`，`StaticChecker` 会在所有非 `MOCK` 模式下强制拦截，防止未经真机验证的焊接指令流出到控制器。

---

## 三、ROS 2 通信接口与支持能力

### 1. ROS 2 接口清单

| 类型 | 名称 | 接口定义 | 说明 |
| --- | --- | --- | --- |
| Action | `/adapter/kuka_arctech/execute_job` | `atomic_adapter_interfaces/action/ExecuteBackendJob` | 提交单条或多步原子指令作业，回传实时 `Feedback`、`artifact_manifest_json` 与包含全量 N 条 `step_results[]` 的 `result_json` |
| Topic | `/adapter/kuka_arctech/status` | `atomic_adapter_interfaces/msg/BackendStatus` | 500ms 周期发布控制器连接状态、当前运行模式、焊缝/电弧状态与故障码 |
| Service | `/adapter/kuka_arctech/stop_job` | `atomic_adapter_interfaces/srv/StopBackendJob` | 请求停止指定作业；发生 `KA_STOP_FAILED` 锁槽时通过本服务重试停机恢复 |

### 2. 支持的 7 类原子指令

| `instruction_id` | `capability_id` | 说明 |
| --- | --- | --- |
| `ROB-010` | `robot.motion.move_joint` | 关节运动（ROS `rad` -> KRL `deg` `E6AXIS` / `SPTP`，支持按 `joint_names` 映射 `A1..A6`） |
| `ROB-011` | `robot.motion.move_linear` | 笛卡尔直线运动（ROS `m` + 四元数 -> KRL `mm` + ZYX 欧拉角 `E6POS` / `SLIN`） |
| `ROB-015` | `robot.motion.follow_path` | 多路点连续笛卡尔轨迹跟随（展开为多段 `SLIN`） |
| `WLD-003` | `welder.recipe.select` | 焊接配方选择（通过 `recipe_wdat_map` 映射并校验 `WDAT` 白名单） |
| `WLD-015` | `welder.arc.start` | 起弧请求（校验 `lease_ms` 并执行 PLC `seam_id` 许可握手） |
| `WLD-016` | `welder.arc.stop` | 收弧请求（强制与 `WLD-015` 成对闭合校验） |
| `WLD-024` | `welder.measure.get_process` | 焊接电流/电压过程量采样（回传于对应 `step_results[i].payload.readings`） |

---

## 四、构建与单元测试

环境要求：Ubuntu 22.04、ROS 2 Humble、C++17 编译器、CMake 3.12+、`colcon`。

```zsh
# 1. 编译工作区（启用 --symlink-install，改配置/模板/launch 无需重编）
./build.sh

# 2. 加载环境（Bash 用户请替换为 setup.bash）
source /opt/ros/humble/setup.zsh
source install/setup.zsh

# 3. 运行自动化单元测试套件 (KA-T01 ~ KA-T09)
colcon test --packages-select kuka_arctech_adapter --event-handlers console_direct+
colcon test-result --all
```

---

## 五、启动与联调指南

### 1. 启动 B 组 Adapter

```zsh
# 方式 A：Mock 联调模式（默认，走完整校验 + 模板渲染 + 静态检查 + 内存模拟执行）
ros2 launch kuka_arctech_adapter kuka_arctech_adapter.launch.py transport_mode:=mock

# 方式 B：Manual 人工导出模式（校验通过后将 .src/.dat/manifest 原子导出到 output/）
ros2 launch kuka_arctech_adapter kuka_arctech_adapter.launch.py transport_mode:=manual
```

`kuka_arctech_adapter.launch.py` 参数说明：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `node_name` | `kuka_arctech_adapter` | 节点名称 |
| `log_level` | `info` | 日志级别（`debug` / `info` / `warn` / `error` / `fatal`） |
| `transport_mode` | `mock` | 传输后端：`mock` 或 `manual`（`verified` 待现场控制器协议确认后接入） |
| `manual_export_dir` | `output` | `manual` 模式下的程序包导出根目录 |
| `templates_dir` | `""` | 自定义黄金模板目录覆盖路径（为空时默认使用安装目录下的 `share/templates`） |
| `debug_dir` | `debug` | 全链路 JSONL 调试记录目录（传入空字符串 `""` 可关闭记录） |

### 2. 使用 `sim_A` 发送单步 / 多步预设作业

`sim_A` 内置了 **4 个单步预设** 与 **4 个多步连续运动预设**（如矩形巡航、多点关节往返、双段轨迹空跑、8 步标准演示），完整列表与参数说明见 [`src/sim_A/README.md`](src/sim_A/README.md)：

```zsh
# 默认发送 8 步空跑焊接联调作业 (job_8steps_demo.json)
ros2 launch sim_A sim_a.launch.py

# 发送 4 步矩形巡航多指令作业
ros2 launch sim_A sim_a.launch.py job_file:=job_multi_motion_square.json

# 发送纯 WLD-024 只读测量查询作业
ros2 launch sim_A sim_a.launch.py job_file:=job_single_wld024.json

# 模拟执行中途取消（发送后 0.3 秒触发 Cancel）
ros2 launch sim_A sim_a.launch.py job_file:=job_multi_motion_square.json cancel_after_sec:=0.3
```

### 3. 查看后端状态、调用停止服务与组件加载

```zsh
# 订阅后端实时状态
ros2 topic echo /adapter/kuka_arctech/status

# 调用 StopBackendJob 主动停止当前作业（或解除 KA_STOP_FAILED 锁槽）
ros2 service call /adapter/kuka_arctech/stop_job \
  atomic_adapter_interfaces/srv/StopBackendJob \
  '{job_id: "job_mock_002"}'

```

### 4. 查看全链路调试记录（`debug/*.jsonl`）

每次启动 Adapter 会在 `debug/` 下生成按时间命名的 JSONL 审计日志，记录 `action_goal`、`action_feedback`、`action_cancel`、`action_result`、`stop_request`、`stop_response` 及状态跳变时的 `backend_status`：

```zsh
jq -c 'select(.job_id == "job_mock_002" or .current_job_id == "job_mock_002")' debug/*.jsonl
```

---

## 六、标准错误码与当前里程碑

### 1. 错误码定义（`ExecuteBackendJob.Result.error_code`）

成功时为空字符串 `""`，失败时严格使用以下 11 个标准错误码：

| 错误码 | 触发场景 |
| --- | --- |
| `KA_JOB_INVALID` | Goal 字段缺失、JSON 格式非法、限位超界、指令时序非法或无法映射到 KUKA IR |
| `KA_TEMPLATE_NOT_FOUND` | 找不到匹配目标版本或工艺类型的黄金模板 |
| `KA_TEMPLATE_RENDER_FAILED` | 模板渲染失败（如存在未声明占位符或展开异常） |
| `KA_STATIC_CHECK_FAILED` | 生成的 KRL 未通过静态检查，或在非 Mock 模式下命中未验证模板安全门 |
| `KA_KRC_UNREACHABLE` | 无法连接或探测 KRC（含 Manual 模式执行纯在线查询） |
| `KA_DEPLOY_FAILED` | 程序包部署或本地导出失败 |
| `KA_PROGRAM_START_FAILED` | 控制器选程序/启动失败，或 Manual 模式导出后等待人工在示教器启动 |
| `KA_SEAM_PERMISSION_TIMEOUT` | 等待 PLC 焊缝许可握手超时或授予的 `seam_id` 不匹配 |
| `KA_ARCTECH_FAULT` | ArcTech 焊接过程故障（保留控制器原始故障证据） |
| `KA_PROGRAM_TIMEOUT` | 单步或作业运行超过 `timeout_ms` 并已触发停机 |
| `KA_STOP_FAILED` | 停机未获确认，自动锁定作业槽位并持续上报故障，需人工确认或调用 `stop_job` 重试解锁 |

### 2. 当前进度

- **已完成**：
  - 路线 A（ROS 2 契约闭环、7 类指令映射校验、单槽位互斥与 `KA_STOP_FAILED` 锁槽保护、`MockKrcTransport`）
  - 路线 B（KSS 8.x `dry_run_motion` 黄金模板引擎、`StaticChecker` 安全门与 SHA-256 清单、`ManualKrcTransport` 原子落盘导出）
  - `utils/` 工具层解耦、`DebugRecorder` JSONL 审计日志、`sim_A` 多场景测试集、自动化单测 `KA-T01 ~ KA-T09`（全绿通过）
- **待现场条件具备后接入**：
  - 在真实 KRC 示教器上导出含 `ARCON` / `ARCOFF` / `WDAT` 的验证版焊接模板，替换 `straight_weld` 占位模板
  - 实现连接真实 KRC / PLC 的 `VerifiedKrcTransport`（程序自动部署启停 + 实时电流电压采集）及现场验收测试 `KA-T10 ~ KA-T12`

> **注意**：目前 `result_json.step_results` 中的观测数据（实际关节角与位姿、误差、电弧状态、电流电压等）全部是模拟值或目标值回填，每步完成也是靠计时判定，不是靠控制器反馈。逐字段清单、结构上缺失的接口以及上真机前要修的 KRL 问题，见 [真机接入清单](docs/hardware_integration.md)。
