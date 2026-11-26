# 真机接入清单：模拟数据与待实现项

本文列出 `kuka_arctech_adapter` 中尚未接入真实设备的部分，供上真机调试时逐项替换。代码位置以 2026-11-26 的提交为准，行号可能随后续修改漂移，以函数名为准。

## 1. 现状总览

| 环节 | 状态 | 说明 |
| --- | --- | --- |
| Goal 校验、语义映射、IR 校验 | 已实现 | 处理真实输入 |
| 模板渲染、静态检查、SHA-256 清单 | 已实现 | 生成真实文件，但程序本身有待修问题，见第 5 节 |
| Manual 导出 | 已实现 | 不伪造执行结果，步骤状态为 `WAITING_OPERATOR` |
| 取消、超时、停机失败锁槽等状态机 | 已实现 | 通过 Mock 故障注入测试 |
| 执行过程与 `step_results` 中的观测数据 | **全部模拟** | 见第 2、3 节 |
| `VerifiedKrcTransport` | 未开始 | `transport_mode:=verified` 会拒绝启动 |

## 2. `step_results` 中的模拟字段

全部由 [`build_mock_step_output()`](../src/kuka_arctech_adapter/utils/src/result_json.cpp#L70) 根据 IR 中的**目标值**生成，与控制器无关。

| 指令 | 字段 | 当前实现 | 真实数据来源 |
| --- | --- | --- | --- |
| ROB-010 | `payload.actual_rad` | 回填目标角度 `target_rad` | KRC 实际轴角 `$AXIS_ACT`（deg，需换算成 rad，并按 `joint_names` 反向映射） |
| ROB-010 | `payload.max_error_rad` | 固定 `0.0` | 实际值与目标值逐轴求差取最大 |
| ROB-010 | `payload.settled` | 固定 `true` | 由连续采样判定：误差小于 `joint_tolerance_rad`，速度小于 `settle_velocity_rad_s`，并持续 `settle_duration_ms` |
| ROB-010 | `evidence.settle_duration_ms` | 回填请求值 | 实际测得的稳定持续时间 |
| ROB-010 | `evidence.settle_velocity_rad_s` | 固定 `0.0` | KRC 轴速度反馈，或由相邻采样差分估算 |
| ROB-011 | `payload.actual_pose` | 回填目标位姿 | KRC 实际 TCP 位姿 `$POS_ACT`（mm/deg，需经 `kuka_to_ros_pose` 换算） |
| ROB-011 | `payload.position_error_m` / `orientation_error_rad` | 固定 `0.0` | 实际位姿与目标位姿求差 |
| ROB-011 | `payload.max_path_deviation_m` | 固定 `0.0` | 运动过程中采样实际轨迹，计算到理想直线的最大距离 |
| ROB-011 | `payload.settled`、`evidence.settle_duration_ms` | 固定 `true`、回填请求值 | 同 ROB-010 的稳定判定 |
| ROB-015 | `payload.completed_waypoints` | 等于输入路径点数 | 控制器实际走过的路径点数（中途停止时会小于总数） |
| ROB-015 | `payload.actual_pose` | 回填最后一个路径点 | KRC 实际 TCP 位姿 |
| ROB-015 | `payload.max_path_deviation_m` | 固定 `0.0` | 实际轨迹采样 |
| WLD-003 | `payload.active_program_id` | 回显请求的 `program_id` | 焊机或 PLC 回读的当前配方号 |
| WLD-015 | `payload.arc_established` | 固定 `true` | ArcTech 或 PLC 的电弧建立信号 |
| WLD-016 | `payload.arc_stopped` | 固定 `true` | ArcTech 或 PLC 的电弧熄灭信号 |
| WLD-024 | `readings[current].value` | 固定 `182.4` A | 焊机或 PLC 采集 |
| WLD-024 | `readings[voltage].value` | 固定 `22.7` V | 焊机或 PLC 采集 |
| WLD-024 | `readings[*].valid` | 固定 `true` | 由采集链路判断数据是否有效 |
| 全部 | `observed_at` | Adapter 本机 UTC 时间 | 控制器或采集设备的采样时间戳 |
| 全部 | `evidence.simulated` | `mode == "MOCK"` 时为 true | 见 3.3 |

## 3. 结构上缺失的部分

### 3.1 `IKrcTransport` 没有读取观测数据的接口

[`krc_transport.h`](../src/kuka_arctech_adapter/include/krc_transport.h) 现有方法只有 `probe`、`deploy`、`select_program`、`start_program`、`stop_program`、`read_status`、`set_seam_permission`、`read_faults`。`read_status()` 返回的 `KrcStatus` 只有几个状态字符串，没有任何数值。

接入真机前需要新增的能力（命名仅供参考）：

| 能力 | 用于 |
| --- | --- |
| 读取实际轴角与轴速度 | ROB-010 的 `actual_rad`、误差和稳定判定 |
| 读取实际 TCP 位姿 | ROB-011/015 的 `actual_pose` 和误差 |
| 读取程序执行进度（当前在第几步） | 见 3.2 |
| 读取焊接过程量（电流、电压） | WLD-024 |
| 读取电弧状态和当前配方 | WLD-003/015/016 |

数据来自哪条链路（RSI、EKI、OPC UA、PLC）要等现场确定后再选，这会决定接口是同步读取还是订阅推送。

### 3.2 步骤完成是靠计时，不是靠控制器反馈

[`ProgramExecutor::execute()`](../src/kuka_arctech_adapter/src/program_executor.cpp#L371) 在 `start_program()` 之后逐步遍历 IR。每一步只是睡眠 `step_delay_ms_`（默认 50 ms，见 [program_executor.h:60](../src/kuka_arctech_adapter/include/program_executor.h#L60)），然后调用 `make_mock_step_json` 标记成功（[program_executor.cpp:425](../src/kuka_arctech_adapter/src/program_executor.cpp#L425)）。

真机上，KRC 会一次性运行整个 `.src`，Adapter 需要知道程序当前执行到哪一步。可选做法：

- 在每步运动前后生成写全局变量或输出位的 KRL 语句（例如写一个步骤计数变量），Adapter 轮询这个值。这需要同步修改模板引擎和静态检查的白名单。
- 读取程序指针（当前行号），再与生成时记录的“步骤到行号”映射表对照。

`timeout_ms` 超时检测目前也在这个睡眠循环里，改成轮询后要一并迁移。

### 3.3 执行器在任何模式下都生成模拟结果

`make_mock_step_json` 在执行器中被无条件调用（[program_executor.cpp:168](../src/kuka_arctech_adapter/src/program_executor.cpp#L168) 只读查询路径，[:425](../src/kuka_arctech_adapter/src/program_executor.cpp#L425) 运动路径）。而 `evidence.simulated` 只在 `mode == "MOCK"` 时为 true（[result_json.cpp:44](../src/kuka_arctech_adapter/utils/src/result_json.cpp#L44)）。

**风险**：如果新增 Verified 模式但不改执行器，A 组会收到 `simulated: false`，但数据仍然是伪造的。接入时必须做到：

1. `make_mock_step_json` 只在 Mock 模式下调用。
2. 其他模式由真实采样结果构造 payload。
3. 采不到数据时，返回失败或 `valid: false`，不能回退成目标值。

### 3.4 运行状态由执行器写入，不是从控制器读取

执行器通过 `set_runtime_state()` 主动设置 `RUNNING`/`WELDING`/`IDLE`（例如 [program_executor.cpp:429](../src/kuka_arctech_adapter/src/program_executor.cpp#L429)），`/adapter/kuka_arctech/status` 发布的就是这个值。接入真机后，`program_state` 和 `weld_state` 应以控制器回读为准，`set_runtime_state` 在 Verified 实现中应为空操作（接口注释已有此约定）。

### 3.5 焊接安全相关逻辑不完整

| 问题 | 位置 | 说明 |
| --- | --- | --- |
| 焊缝许可是模拟握手 | `MockKrcTransport::set_seam_permission` | Mock 直接授予；Manual 一律拒绝 |
| 焊缝许可从不收回 | [program_executor.cpp:313](../src/kuka_arctech_adapter/src/program_executor.cpp#L313) 是唯一调用，参数固定为 `true` | 收弧、失败、取消后都没有调用 `set_seam_permission(seam_id, false)` |
| `lease_ms` 只做了校验，没有执行 | `job_validator.cpp` 只检查非零 | 没有租约计时，到期也不会自动收弧或停机 |
| 故障只在每步开始前检查 | [program_executor.cpp:290](../src/kuka_arctech_adapter/src/program_executor.cpp#L290) | 步骤执行中发生的故障要等到下一步才会发现 |
| 停机结果是模拟的 | `MockKrcTransport::stop_program` | 真机需回读程序状态，确认已停止后才返回成功 |

## 4. Mock 模式可注入的故障

`MockTransportOptions`（[mock_krc_transport.h](../src/kuka_arctech_adapter/include/mock_krc_transport.h)）支持以下开关，目前只在单元测试中使用，launch 参数不能设置：

| 选项 | 效果 |
| --- | --- |
| `reachable=false` | 探活、部署、启停都返回 `KA_KRC_UNREACHABLE` |
| `fail_deploy` | 部署返回 `KA_DEPLOY_FAILED` |
| `fail_start` | 启动返回 `KA_PROGRAM_START_FAILED` |
| `fail_stop` | 停止返回 `KA_STOP_FAILED`，触发锁槽 |
| `deny_seam_permission` | 焊缝许可返回 `KA_SEAM_PERMISSION_TIMEOUT` |
| `injected_fault_code` / `injected_fault_message` | `read_faults()` 返回该故障，触发 `KA_ARCTECH_FAULT` |

## 5. 生成的 KRL 程序在上真机前需要处理的问题

这些问题在 Mock 和 dry run 下看不出来，但会影响真机运行。

| 优先级 | 问题 | 位置 | 建议 |
| --- | --- | --- | --- |
| 高 | 外部字符串原样写进 KRL 注释，换行符可以注入代码 | `TemplateEngine::render` 中的 `JOB_ID`、`TRACE_ID`、`instance_id`、`start_profile_id`、`crater_profile_id` | validator 中限制为可打印 ASCII，并禁止 `{`、`}` |
| 高 | ROB-015 路径点之间没有圆滑过渡，每点都会精确暂停，实焊时会造成焊缝缺陷 | `FollowPathStep` 分支生成的 `SLIN` 没有 `C_SPL` | 中间点加 `C_SPL`，或改用 `SPLINE ... ENDSPLINE`；同时把 LDAT 里照抄的 `APO_DIST 500` 改成合理值 |
| 高 | 只含 ROB-011/015 的作业第一条运动是 SLIN，真机上会触发 BCO 运行 | 模板中去掉了真机导出里的 `SPTP XHOME` | 要求第一条运动必须是 SPTP，或在模板首尾加回 HOME |
| 中 | 姿态和关节零位约定未与 A 组确认 | `ros_to_kuka_pose`、`resolve_krl_axis_values` | 确认工具作业方向（KUKA 为 +X，ROS 常用 +Z）以及轴角零位和正方向 |
| 中 | `dry_run_motion` 标记为 `verified_on_krc: true`，但改写后的模板本身没在 KRC 上加载过 | `template_manifest.json` | 先导入一份生成程序到示教器，确认能加载、能空跑，再保留 true |
| 低 | ROB-015 的 LDAT 加速度写死 100%，ROB-011 按输入计算 | `TemplateEngine::render` | 统一按输入或配置取值 |
| 低 | 路径首点与当前目标重合时会生成零长度运动段 | 语义映射 | 丢弃与上一个目标重合的首个路径点 |
| 低 | 不同 `job_id` 换算后可能得到同名程序，Manual 导出会互相覆盖 | `SemanticMapper::make_program_name` | 冲突时追加序号或短哈希 |
| 低 | `straight_weld` 是占位模板，没有真实 ARCON/ARCOFF/WDAT | `templates/kss8_arctech38/straight_weld/` | 在示教器上录制焊接程序后替换 |

## 6. 建议的接入顺序

1. 修第 5 节中的高优先级问题，用 Manual 模式导出一份程序，在示教器上加载并以 T1 低速空跑。
2. 确定数据链路，扩充 `IKrcTransport`（3.1），实现 `VerifiedKrcTransport` 的探活、部署、启停和状态回读。
3. 改造执行器：用控制器进度替代计时（3.2），模拟结果只用于 Mock（3.3），状态改为从控制器读取（3.4）。
4. 接入焊接链路：焊缝许可的授予与收回、`lease_ms` 租约、过程量采集（3.5）。
5. 录制真实焊接黄金模板，替换 `straight_weld`，再做实焊验收（KA-T10 ~ KA-T12）。
