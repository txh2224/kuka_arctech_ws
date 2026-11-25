# kuka_arctech_ws

面向 KUKA ArcTech 的 ROS 2 Humble / C++17 工作区。Adapter 接收 A 组提交的 `AtomicInstruction` 序列，目标是完成 KUKA 专用校验、生成 `SRC/DAT` 程序包，并回传作业状态与结果。

## 当前进度

`AdapterNode` 已接通 Action 接单、工作线程、请求校验、结果返回和组件卸载。节点一次接一单，空闲时先接受 Goal，再检查外层字段。校验失败返回 `KA_JOB_INVALID` 和具体原因。

模板生成、程序包校验、KRC transport、状态发布和停止服务待接入。

| 包 | 职责 |
| --- | --- |
| [`atomic_adapter_interfaces`](src/atomic_adapter_interfaces) | 定义 `AtomicInstruction`、`ExecuteBackendJob`、`BackendStatus` 和 `StopBackendJob` |
| [`kuka_arctech_adapter`](src/kuka_arctech_adapter) | 提供 `AdapterNode` 组件和 `adapter_node` 独立启动入口 |

## 构建

需要 ROS 2 Humble、支持 C++17 的编译器、CMake 3.12 或更高版本和 `colcon`。在工作区根目录运行构建脚本，默认使用 Release 配置。 [`build.sh`](build.sh)

```zsh
./build.sh
source install/setup.zsh
```

使用 Bash 时改为 `source install/setup.bash`。每个运行 ROS 2 命令的新终端都需要加载工作区环境。

## 启动

可以单独运行节点。

```sh
ros2 run kuka_arctech_adapter adapter_node
```

也可以在第一个终端启动组件容器。

```sh
ros2 run rclcpp_components component_container
```

在第二个终端加载组件，命令会打印组件 ID。

```sh
ros2 component load /ComponentManager kuka_arctech_adapter AdapterNode
```

## IR 前检查

- `job_id` 和 `trace_id` 都不能为空。
- `backend` 必须为 `kuka_arctech`。
- `instructions` 至少包含一条指令。

指令类型、JSON 参数结构和字段类型在 IR 转换时处理。位姿与速度范围、路径点数、焊接顺序和 WDAT 配方关系由后续 IR 校验负责。

## 验证 Action

节点注册的相对 Action 名称是 `execute_backend_job`，默认命名空间下的完整路径是 `/execute_backend_job`。启动节点后，在另一个终端发送两条测试请求。

```sh
ros2 action send_goal /execute_backend_job \
  atomic_adapter_interfaces/action/ExecuteBackendJob \
  '{job_id: demo_invalid, trace_id: trace_001, backend: kuka_arctech, dry_run: true, instructions: []}'

ros2 action send_goal /execute_backend_job \
  atomic_adapter_interfaces/action/ExecuteBackendJob \
  '{job_id: demo_valid_envelope, trace_id: trace_002, backend: kuka_arctech, dry_run: true, instructions: [{instruction_id: smoke_001}]}'
```

第一条请求的指令序列为空，Result 中的 `error_code` 应为 `KA_JOB_INVALID`。第二条包含一条占位指令，通过 Goal 外层检查后进入待接入的执行分支。当前两条请求都会以 `ABORTED` 和 `success: false` 结束，第二条返回空 `error_code`。

使用组件方式启动时，可以通过组件 ID 卸载节点。以下以 ID 为 `1` 为例。

```sh
ros2 component list
ros2 component unload /ComponentManager 1
```

## 错误码

以下是《KUKA ArcTech Adapter 开发任务规格书》第 12 节规定的业务错误码，填入 `ExecuteBackendJob.Result.error_code`。

| 错误码 | 场景 |
| --- | --- |
| `KA_JOB_INVALID` | 作业缺少参数、指令顺序不合法，或无法映射到 KUKA 后端 |
| `KA_TEMPLATE_NOT_FOUND` | 找不到匹配目标版本或工艺的黄金模板 |
| `KA_TEMPLATE_RENDER_FAILED` | 模板实例化失败，例如占位符或格式错误 |
| `KA_STATIC_CHECK_FAILED` | 生成包未通过静态校验，阻止部署 |
| `KA_KRC_UNREACHABLE` | 无法连接或探测 KRC |
| `KA_DEPLOY_FAILED` | 程序包部署失败，保留原始错误证据 |
| `KA_PROGRAM_START_FAILED` | 控制器选择或启动程序失败，保留原始错误证据 |
| `KA_SEAM_PERMISSION_TIMEOUT` | 等待焊缝许可握手超时 |
| `KA_ARCTECH_FAULT` | ArcTech 焊接故障，保留控制器原始故障编号和消息 |
| `KA_PROGRAM_TIMEOUT` | 程序运行超时，应请求停止 |
| `KA_STOP_FAILED` | 停止未获确认，应进入 `UNKNOWN` 或 `FAILED` 并提示人工检查 |

规格书附录 A 约定顶层 `error_code` 使用标准 `KA_*` 码，控制器的原始编号、消息和现场信息可放在 `result_json` 的 `evidence` 中。

## 后端联调准备

- 从 A 组取得实际 Goal 样例，确认各 `*_json` 字段、指令顺序和单位，再完成 KUKA 专用校验与 IR 映射。
- 从目标控制器取得经现场验证的黄金 `SRC/DAT`，配套记录系统与 ArcTech 版本、TOOL/BASE、WDAT 和允许替换的模板字段。
- 先用 Mock transport 验证程序包、状态机和故障路径，再准备 Manual transport 的人工导入流程。接入真实 KRC 前确认已授权的部署和控制接口、PLC 点表及任务终态的确认方式。
