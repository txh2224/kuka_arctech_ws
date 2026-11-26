# sim_A

`sim_A` 是联调时模拟 A 组发单的 ROS 2 Action 客户端。每次启动读取一份作业 JSON，把其中的单条或多条原子指令作为一个 `ExecuteBackendJob` Goal 发送给 Adapter，并在终端打印接单状态、执行进度和结果。

## 构建和运行

使用 ROS 2 Humble。在工作区根目录构建一次，并在两个终端分别加载工作区环境。Bash 用户把 `setup.zsh` 换成 `setup.bash`。

```zsh
./build.sh
source install/setup.zsh
```

第一个终端启动 Adapter 的 Mock 模式。

```zsh
ros2 launch kuka_arctech_adapter kuka_arctech_adapter.launch.py transport_mode:=mock
```

第二个终端运行默认八步作业，或指定一个单步样例。

```zsh
ros2 launch sim_A sim_a.launch.py
ros2 launch sim_A sim_a.launch.py job_file:=job_single_rob010.json
```

## 启动参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `job_file` | `job_8steps_demo.json` | 作业文件名或路径。先检查给定路径，再从已安装的 `sim_A/config/` 查找。 |
| `action_name` | `/adapter/kuka_arctech/execute_job` | Adapter 的 `ExecuteBackendJob` Action 端点。 |
| `wait_server_sec` | `5.0` | 等待发现 Action Server 的时长，单位为秒。 |
| `cancel_after_sec` | `0.0` | Goal 被受理后等待多少秒再发 Action Cancel。设为 `0.0` 关闭定时取消。 |
| `auto_shutdown` | `true` | 收到最终结果、请求被拒绝或发单失败后自动退出。共享组件容器内运行时设为 `false`。 |

命令行传秒数时使用小数写法，例如 `wait_server_sec:=8.0`。测试取消时可用八步作业，以下命令会在 Goal 受理约 100 ms 后请求取消。如果作业提前结束，客户端会取消定时器。

```zsh
ros2 launch sim_A sim_a.launch.py cancel_after_sec:=0.1
```

如需测 Adapter 的 `StopBackendJob` 服务，在作业运行中用实际 `job_id` 调用服务。服务的 `accepted` 表示请求受理，最终状态看 Action Result 或状态话题。具体命令见[工作区 README 的停止服务示例](../../README.md#3-查看状态与调用停止服务)。

## 内置样例

| 文件 | 步数 | 覆盖场景 |
| --- | ---: | --- |
| [job_8steps_demo.json](config/job_8steps_demo.json) | 8 | 默认作业，按序覆盖七种能力，包含两次 `ROB-011`。 |
| [job_single_rob010.json](config/job_single_rob010.json) | 1 | 六轴关节运动。 |
| [job_single_rob011.json](config/job_single_rob011.json) | 1 | 单目标笛卡尔直线运动。 |
| [job_single_rob015.json](config/job_single_rob015.json) | 1 | 多路点笛卡尔路径。 |
| [job_single_wld024.json](config/job_single_wld024.json) | 1 | 查询模拟焊接电流和电压。 |
| [job_multi_joint_sweep.json](config/job_multi_joint_sweep.json) | 3 | 同一 Goal 中连续执行三条关节运动。 |
| [job_multi_motion_square.json](config/job_multi_motion_square.json) | 4 | 单批作业包含关节运动、直线段和方形路径。 |
| [job_multi_2seams_dryrun.json](config/job_multi_2seams_dryrun.json) | 7 | 同一 Goal 中走两段焊缝路径，中间有转场，后续查询一次过程量。 |

样例均设置 `dry_run=true`。自定义作业可以复制任意样例，设置 `job_id`、`trace_id` 和 `instructions` 后，通过 `job_file:=/绝对路径/my_job.json` 发单。指令里的 `*_json` 字段可以直接写 JSON 对象，也可以写序列化后的字符串。客户端会把对象序列化为 ROS 接口使用的字符串。

## 查看结果

- `[Feedback]` 输出 `stage`、从 0 开始的步骤索引 `index`、进度百分比、指令 ID 和消息。步骤重复时用 `index` 定位所在位置。
- `[Result]` 输出 Action 终态、业务 `success`、`error_code` 和 `backend_job_id`。随后会打印 `result_json`，有产物时还会打印 `artifact_manifest_json`。用 `step_results` 中的 `index` 和 `instance_id` 对照输入。
- Adapter 若以 Manual 模式导出运动程序包，可用 `result_json.manual_handoff_required=true` 和 `export_dir` 确认交接。这个阶段会显示 `success=false` 和 `KA_PROGRAM_START_FAILED`，提示等待人工导入并启动 KRC 程序。
- 需要核对 ROS 实际收到的 Goal、反馈和结果时，在 Adapter 的 `debug/*.jsonl` 中按 `job_id` 或 `trace_id` 查找。操作见[查看调试记录](../../README.md#5-查看调试记录)。
