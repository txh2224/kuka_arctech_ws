#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "error_code.h"
#include "job_ir.h"
#include "job_validator.h"
#include "manual_krc_transport.h"
#include "mock_krc_transport.h"
#include "program_executor.h"
#include "semantic_mapper.h"
#include "static_checker.h"
#include "template_engine.h"

namespace {

using ExecuteGoal = atomic_adapter_interfaces::action::ExecuteBackendJob::Goal;
using AtomicInst = atomic_adapter_interfaces::msg::AtomicInstruction;

AtomicInst make_joint_inst(
    const std::string &inst_id = "inst_j1",
    const std::string &joint_names_json = R"(["A1","A2","A3","A4","A5","A6"])",
    const std::string &target_rad_json = R"([0.1, -0.5, 1.0, 0.0, 0.5, 0.0])",
    double max_vel = 1.0,
    double max_acc = 5.0,
    uint32_t timeout_ms = 0,
    const std::string &constraint_json = "") {
    AtomicInst inst;
    inst.instruction_id = "ROB-010";
    inst.instance_id = inst_id;
    inst.capability_id = "robot.motion.move_joint";
    inst.object_json = R"({"robot_id":"kuka_01"})";
    inst.parameter_json =
        std::string(R"({"joint_names":)") + joint_names_json +
        R"(,"target_rad":)" + target_rad_json +
        R"(,"max_velocity_rad_s":)" + std::to_string(max_vel) +
        R"(,"max_acceleration_rad_s2":)" + std::to_string(max_acc) + "}";
    inst.constraint_json = constraint_json;
    inst.timeout_ms = timeout_ms;
    return inst;
}

AtomicInst make_linear_inst(const std::string &inst_id = "inst_l1", double acc = 2.5) {
    AtomicInst inst;
    inst.instruction_id = "ROB-011";
    inst.instance_id = inst_id;
    inst.capability_id = "robot.motion.move_linear";
    inst.object_json = R"({"tool_id":"welding_torch_01","frame_id":"base"})";
    inst.parameter_json =
        R"({"target_pose":{"position_m":[0.45,0.1,0.35],"quaternion_xyzw":[0.0,0.0,0.0,1.0]},"speed_m_s":0.12,"acceleration_m_s2":)" +
        std::to_string(acc) + "}";
    return inst;
}

AtomicInst make_recipe_inst(const std::string &program_id = "WELD_PROGRAM_01") {
    AtomicInst inst;
    inst.instruction_id = "WLD-003";
    inst.instance_id = "inst_recipe";
    inst.capability_id = "welder.recipe.select";
    inst.object_json = R"({"object_id":"welder"})";
    inst.parameter_json = std::string(R"({"program_id":")") + program_id + R"("})";
    return inst;
}

AtomicInst make_arc_start_inst() {
    AtomicInst inst;
    inst.instruction_id = "WLD-015";
    inst.instance_id = "inst_arc_on";
    inst.capability_id = "welder.arc.start";
    inst.object_json = R"({"object_id":"welder"})";
    inst.parameter_json = R"({"start_profile_id":"START_01","lease_ms":5000})";
    return inst;
}

AtomicInst make_arc_stop_inst() {
    AtomicInst inst;
    inst.instruction_id = "WLD-016";
    inst.instance_id = "inst_arc_off";
    inst.capability_id = "welder.arc.stop";
    inst.object_json = R"({"object_id":"welder"})";
    inst.parameter_json = R"({"crater_profile_id":"CRATER_01"})";
    return inst;
}

AtomicInst make_measure_inst() {
    AtomicInst inst;
    inst.instruction_id = "WLD-024";
    inst.instance_id = "inst_measure";
    inst.capability_id = "welder.measure.get_process";
    inst.object_json = R"({"object_id":"welder"})";
    inst.parameter_json = R"({"channels":["current","voltage"]})";
    return inst;
}

ExecuteGoal make_goal(const std::string &job_id, bool dry_run = true) {
    ExecuteGoal g;
    g.job_id = job_id;
    g.trace_id = "trace_" + job_id;
    g.backend = "kuka_arctech";
    g.dry_run = dry_run;
    return g;
}

std::string templates_root_path() {
    return std::string(KUKA_TEST_SOURCE_DIR) + "/templates";
}

std::string validator_config_path() {
    return std::string(KUKA_TEST_SOURCE_DIR) + "/config/validator.json";
}

}  // namespace

// KA-T01: 合法单步与只读查询路径 + 约束 settle_duration_ms 透传
TEST(KukaArcTechAdapterTest, KA_T01_ValidMotionAndReadOnlyQuery) {
    JobValidator validator;
    SemanticMapper mapper;
    TemplateEngine engine(templates_root_path());
    StaticChecker checker(validator_config_path());

    // 1. 单步 ROB-010 带自定义 settle_duration_ms
    auto goal = make_goal("t01_joint", true);
    goal.instructions.push_back(make_joint_inst(
        "inst_j1",
        R"(["A1","A2","A3","A4","A5","A6"])",
        R"([0.1,-0.5,1.0,0.0,0.5,0.0])",
        1.0,
        5.0,
        0,
        R"({"settle_duration_ms":350})"));

    std::string err_msg;
    ASSERT_EQ(validator.validate(goal, err_msg), ErrorCode::OK) << err_msg;

    KukaArcTechJobIR ir;
    ASSERT_EQ(mapper.map(goal, ir, err_msg), ErrorCode::OK) << err_msg;
    ASSERT_EQ(validator.validate_ir(ir, err_msg), ErrorCode::OK) << err_msg;
    EXPECT_EQ(ir.steps[0].settle_duration_ms, 350u);

    ProgramPackage pkg;
    ASSERT_EQ(engine.render(ir, pkg, err_msg), ErrorCode::OK) << err_msg;
    ASSERT_EQ(checker.check_and_build_manifest(ir, "MOCK", pkg, err_msg), ErrorCode::OK) << err_msg;

    auto transport = std::make_shared<MockKrcTransport>();
    ProgramExecutor executor(transport, 10);
    std::atomic<bool> stop_flag{false};
    std::vector<float> progresses;
    const auto res = executor.execute(
        ir,
        stop_flag,
        [&](uint8_t, uint32_t, float p, const std::string &, const std::string &) {
            progresses.push_back(p);
        },
        &pkg);
    ASSERT_EQ(res.outcome, ExecutionOutcome::SUCCEEDED);
    ASSERT_EQ(res.error_code, ErrorCode::OK);

    // 验证进度单调非递减
    for (std::size_t i = 1; i < progresses.size(); ++i) {
        EXPECT_GE(progresses[i], progresses[i - 1]);
    }

    const auto doc = nlohmann::json::parse(res.result_json);
    ASSERT_EQ(doc.at("step_results").size(), 1u);
    EXPECT_EQ(doc.at("step_results")[0].at("evidence").at("settle_duration_ms").get<uint32_t>(), 350u);

    // 2. 纯只读测量 WLD-024 走查询路径
    auto q_goal = make_goal("t01_query", true);
    q_goal.instructions.push_back(make_measure_inst());
    KukaArcTechJobIR q_ir;
    ASSERT_EQ(mapper.map(q_goal, q_ir, err_msg), ErrorCode::OK) << err_msg;
    ASSERT_EQ(validator.validate_ir(q_ir, err_msg), ErrorCode::OK) << err_msg;
    EXPECT_TRUE(is_read_only_query_job(q_ir));

    std::vector<uint8_t> q_stages;
    const auto q_res = executor.execute(
        q_ir,
        stop_flag,
        [&](uint8_t stage, uint32_t, float, const std::string &, const std::string &) {
            q_stages.push_back(stage);
        },
        nullptr);
    ASSERT_EQ(q_res.outcome, ExecutionOutcome::SUCCEEDED);
    ASSERT_EQ(q_stages.size(), 1u);
    EXPECT_EQ(q_stages[0], static_cast<uint8_t>(ExecutionStage::WELDING));
    const auto q_doc = nlohmann::json::parse(q_res.result_json);
    EXPECT_TRUE(q_doc.value("query_only", false));
    ASSERT_EQ(q_doc.at("step_results").size(), 1u);
    EXPECT_EQ(q_doc.at("step_results")[0].at("state").get<std::string>(), "SUCCEEDED");
}

// KA-T02: 非法作业、非法 JSON、seam_id 超限（>65535）返回 KA_JOB_INVALID
TEST(KukaArcTechAdapterTest, KA_T02_InvalidJobAndSeamIdLimit) {
    JobValidator validator;
    SemanticMapper mapper;
    std::string err_msg;

    // 1. 非法 constraint_json
    auto bad_json_goal = make_goal("t02_bad_json", true);
    auto inst = make_joint_inst();
    inst.constraint_json = "{not_json";
    bad_json_goal.instructions.push_back(inst);
    KukaArcTechJobIR ir;
    EXPECT_EQ(mapper.map(bad_json_goal, ir, err_msg), ErrorCode::KA_JOB_INVALID);

    // 2. 缺少配方直接起弧
    auto no_recipe_goal = make_goal("t02_no_recipe", false);
    no_recipe_goal.instructions.push_back(make_arc_start_inst());
    no_recipe_goal.instructions.push_back(make_arc_stop_inst());
    ASSERT_EQ(mapper.map(no_recipe_goal, ir, err_msg), ErrorCode::OK);
    EXPECT_EQ(validator.validate_ir(ir, err_msg), ErrorCode::KA_JOB_INVALID);

    // 3. seam_id > 65535 超出 KUKA ArcTech 3.8 上限
    auto seam_goal = make_goal("t02_seam_limit", false);
    seam_goal.instructions.push_back(make_recipe_inst());
    seam_goal.instructions.push_back(make_arc_start_inst());
    seam_goal.instructions.push_back(make_arc_stop_inst());
    ASSERT_EQ(mapper.map(seam_goal, ir, err_msg), ErrorCode::OK);
    ir.seam_id = 70000;
    EXPECT_EQ(validator.validate_ir(ir, err_msg), ErrorCode::KA_JOB_INVALID);
}

// KA-T03: 模板缺失与未验证实焊模板在非 MOCK 模式下被拦截
TEST(KukaArcTechAdapterTest, KA_T03_TemplateNotFoundAndUnverifiedGate) {
    SemanticMapper mapper;
    StaticChecker checker(validator_config_path());
    std::string err_msg;

    auto goal = make_goal("t03_weld", false);
    goal.instructions.push_back(make_recipe_inst());
    goal.instructions.push_back(make_arc_start_inst());
    goal.instructions.push_back(make_linear_inst());
    goal.instructions.push_back(make_arc_stop_inst());

    KukaArcTechJobIR ir;
    ASSERT_EQ(mapper.map(goal, ir, err_msg), ErrorCode::OK);

    // 不存在的模板目录 -> KA_TEMPLATE_NOT_FOUND
    TemplateEngine missing_engine("/nonexistent/kuka_templates_dir");
    ProgramPackage pkg;
    EXPECT_EQ(missing_engine.render(ir, pkg, err_msg), ErrorCode::KA_TEMPLATE_NOT_FOUND);

    // 未验证模板（straight_weld verified_on_krc=false）在 MANUAL 模式下被拦截
    TemplateEngine engine(templates_root_path());
    ASSERT_EQ(engine.render(ir, pkg, err_msg), ErrorCode::OK) << err_msg;
    EXPECT_FALSE(pkg.template_verified_on_krc);
    EXPECT_EQ(checker.check_and_build_manifest(ir, "MANUAL", pkg, err_msg), ErrorCode::KA_STATIC_CHECK_FAILED);
    EXPECT_EQ(checker.check_and_build_manifest(ir, "MOCK", pkg, err_msg), ErrorCode::OK);
}

// KA-T04: TemplateEngine 遵守 joint_names 轴映射、请求加速度与无硬编码 XHOME
TEST(KukaArcTechAdapterTest, KA_T04_JointNamesMappingAndAcceleration) {
    SemanticMapper mapper;
    TemplateEngine engine(templates_root_path());
    std::string err_msg;

    // 乱序传入 joint_names：A6 在第一位（1.0 rad ≈ 57.296 deg），A1 在最后一位（0.0 rad）
    auto goal = make_goal("t04_axes", true);
    goal.instructions.push_back(make_joint_inst(
        "inst_j1",
        R"(["A6","A5","A4","A3","A2","A1"])",
        R"([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])",
        1.57,
        5.0));
    goal.instructions.push_back(make_linear_inst("inst_l1", 2.5));

    KukaArcTechJobIR ir;
    ASSERT_EQ(mapper.map(goal, ir, err_msg), ErrorCode::OK) << err_msg;

    ProgramPackage pkg;
    ASSERT_EQ(engine.render(ir, pkg, err_msg), ErrorCode::OK) << err_msg;

    // 不应存在硬编码 XHOME
    EXPECT_EQ(pkg.src_content.find("XHOME"), std::string::npos);
    // A1 应为 0.000，A6 应为 57.296
    EXPECT_NE(pkg.dat_content.find("A1 0.000"), std::string::npos);
    EXPECT_NE(pkg.dat_content.find("A6 57.296"), std::string::npos);
    // 关节加速度 5.0 / 10.0 = 50.000%，直线加速度 2.5 / 5.0 = 50.000%
    EXPECT_NE(pkg.dat_content.find("ACC 50.000"), std::string::npos);
}

// KA-T05: StaticChecker 忽略注释并校验 ARCON/ARCOFF 成对顺序与 SHA-256
TEST(KukaArcTechAdapterTest, KA_T05_StaticCheckerOrderedArcCheckAndSha256) {
    StaticChecker checker(validator_config_path());
    std::string err_msg;

    KukaArcTechJobIR ir;
    ir.job_id = "t05";
    ir.trace_id = "trace_t05";
    ir.dry_run = false;

    ProgramPackage pkg;
    pkg.program_name = "AI_T05";
    pkg.template_id = "test@1.0";
    pkg.template_verified_on_krc = true;
    pkg.dat_content = "DEFDAT AI_T05\r\nENDDAT\r\n";

    // 先收弧后起弧 -> 即使数量相等也必须拦截
    pkg.src_content = "DEF AI_T05( )\r\nARCOFF\r\nARCON\r\nEND\r\n";
    EXPECT_EQ(checker.check_and_build_manifest(ir, "MOCK", pkg, err_msg), ErrorCode::KA_STATIC_CHECK_FAILED);

    // 仅注释里出现 ARCON（dry_run=true）不应误判，且能正常生成 SHA-256
    ir.dry_run = true;
    pkg.src_content = "DEF AI_T05( )\r\n; comment ARCON\r\nEND\r\n";
    ASSERT_EQ(checker.check_and_build_manifest(ir, "MOCK", pkg, err_msg), ErrorCode::OK) << err_msg;
    const auto manifest = nlohmann::json::parse(pkg.manifest_json);
    EXPECT_EQ(manifest.at("src_sha256").get<std::string>(), StaticChecker::sha256_hex(pkg.src_content));
    EXPECT_EQ(manifest.at("dat_sha256").get<std::string>(), StaticChecker::sha256_hex(pkg.dat_content));
}

// KA-T06: 焊缝许可被拒绝 -> KA_SEAM_PERMISSION_TIMEOUT
TEST(KukaArcTechAdapterTest, KA_T06_SeamPermissionDenied) {
    SemanticMapper mapper;
    std::string err_msg;

    auto goal = make_goal("t06_seam", false);
    goal.instructions.push_back(make_recipe_inst());
    goal.instructions.push_back(make_arc_start_inst());
    goal.instructions.push_back(make_arc_stop_inst());

    KukaArcTechJobIR ir;
    ASSERT_EQ(mapper.map(goal, ir, err_msg), ErrorCode::OK);

    MockTransportOptions opts;
    opts.deny_seam_permission = true;
    auto transport = std::make_shared<MockKrcTransport>(opts);
    ProgramExecutor executor(transport, 10);
    std::atomic<bool> stop_flag{false};

    const auto res = executor.execute(ir, stop_flag, nullptr, nullptr);
    EXPECT_EQ(res.outcome, ExecutionOutcome::FAILED);
    EXPECT_EQ(res.error_code, ErrorCode::KA_SEAM_PERMISSION_TIMEOUT);
    const auto doc = nlohmann::json::parse(res.result_json);
    EXPECT_EQ(doc.at("step_results").size(), 3u);
}

// KA-T07: ArcTech 故障注入 -> KA_ARCTECH_FAULT 并保留原始故障码
TEST(KukaArcTechAdapterTest, KA_T07_ArcTechFaultEvidence) {
    SemanticMapper mapper;
    std::string err_msg;

    auto goal = make_goal("t07_fault", true);
    goal.instructions.push_back(make_joint_inst());

    KukaArcTechJobIR ir;
    ASSERT_EQ(mapper.map(goal, ir, err_msg), ErrorCode::OK);

    MockTransportOptions opts;
    opts.injected_fault_code = "ARC_10134";
    opts.injected_fault_message = "Arc ignition lost";
    auto transport = std::make_shared<MockKrcTransport>(opts);
    ProgramExecutor executor(transport, 10);
    std::atomic<bool> stop_flag{false};

    const auto res = executor.execute(ir, stop_flag, nullptr, nullptr);
    EXPECT_EQ(res.outcome, ExecutionOutcome::FAILED);
    EXPECT_EQ(res.error_code, ErrorCode::KA_ARCTECH_FAULT);
    EXPECT_NE(res.result_json.find("ARC_10134"), std::string::npos);
}

// KA-T08: 单步 timeout_ms 超时触发停机 -> KA_PROGRAM_TIMEOUT
TEST(KukaArcTechAdapterTest, KA_T08_StepTimeoutTriggersStop) {
    SemanticMapper mapper;
    std::string err_msg;

    auto goal = make_goal("t08_timeout", true);
    goal.instructions.push_back(make_joint_inst(
        "inst_j1",
        R"(["A1","A2","A3","A4","A5","A6"])",
        R"([0.1,-0.5,1.0,0.0,0.5,0.0])",
        1.0,
        5.0,
        25));  // timeout_ms = 25ms，而单步模拟耗时 120ms

    KukaArcTechJobIR ir;
    ASSERT_EQ(mapper.map(goal, ir, err_msg), ErrorCode::OK);

    auto transport = std::make_shared<MockKrcTransport>();
    ProgramExecutor executor(transport, 120);
    std::atomic<bool> stop_flag{false};

    const auto res = executor.execute(ir, stop_flag, nullptr, nullptr);
    EXPECT_EQ(res.outcome, ExecutionOutcome::FAILED);
    EXPECT_EQ(res.error_code, ErrorCode::KA_PROGRAM_TIMEOUT);
}

// KA-T09: 停机确认、停机失败归并 KA_STOP_FAILED、Manual 模式语义与原子落盘
TEST(KukaArcTechAdapterTest, KA_T09_StopFailedNormalizationAndManualSemantics) {
    SemanticMapper mapper;
    TemplateEngine engine(templates_root_path());
    StaticChecker checker(validator_config_path());
    std::string err_msg;

    auto goal = make_goal("t09_stop", true);
    goal.instructions.push_back(make_joint_inst("inst_1"));
    goal.instructions.push_back(make_joint_inst("inst_2"));

    KukaArcTechJobIR ir;
    ASSERT_EQ(mapper.map(goal, ir, err_msg), ErrorCode::OK);

    // 1. 停机期间断连（stop_program 返回 KA_KRC_UNREACHABLE）也必须归并为 KA_STOP_FAILED 触发锁槽
    auto transport = std::make_shared<MockKrcTransport>();
    ProgramExecutor executor(transport, 80);
    std::atomic<bool> stop_flag{false};

    std::thread stopper([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        MockTransportOptions disconn;
        disconn.reachable = false;
        transport->set_options(disconn);
        stop_flag.store(true);
    });
    const auto stop_res = executor.execute(ir, stop_flag, nullptr, nullptr);
    stopper.join();

    EXPECT_EQ(stop_res.outcome, ExecutionOutcome::FAILED);
    EXPECT_EQ(stop_res.error_code, ErrorCode::KA_STOP_FAILED);

    // 2. Manual 模式：controller_connected=false、ABORTED+KA_PROGRAM_START_FAILED、产物落盘
    const auto export_root =
        std::filesystem::temp_directory_path() / "kuka_arctech_gtest_manual_export";
    std::filesystem::remove_all(export_root);

    ProgramPackage pkg;
    ASSERT_EQ(engine.render(ir, pkg, err_msg), ErrorCode::OK);
    ASSERT_EQ(checker.check_and_build_manifest(ir, "MANUAL", pkg, err_msg), ErrorCode::OK);

    auto manual_transport = std::make_shared<ManualKrcTransport>(export_root.string());
    EXPECT_FALSE(manual_transport->read_status().controller_connected);
    EXPECT_FALSE(manual_transport->probe().connected);
    EXPECT_FALSE(manual_transport->read_status().controller_connected);

    ProgramExecutor manual_exec(manual_transport, 10);
    std::atomic<bool> no_stop{false};
    const auto m_res = manual_exec.execute(ir, no_stop, nullptr, &pkg);
    EXPECT_EQ(m_res.outcome, ExecutionOutcome::FAILED);
    EXPECT_EQ(m_res.error_code, ErrorCode::KA_PROGRAM_START_FAILED);
    const auto m_doc = nlohmann::json::parse(m_res.result_json);
    ASSERT_EQ(m_doc.at("step_results").size(), 2u);
    EXPECT_EQ(m_doc.at("step_results")[0].at("state").get<std::string>(), "SKIPPED");
    EXPECT_EQ(
        m_doc.at("step_results")[0].at("evidence").at("adapter_state").get<std::string>(),
        "WAITING_OPERATOR");
    EXPECT_TRUE(std::filesystem::exists(export_root / pkg.program_name / (pkg.program_name + ".src")));
    EXPECT_TRUE(std::filesystem::exists(export_root / pkg.program_name / (pkg.program_name + ".dat")));
    EXPECT_TRUE(std::filesystem::exists(export_root / pkg.program_name / "artifact_manifest.json"));

    std::filesystem::remove_all(export_root);
}
