#include "template_engine.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

std::string default_templates_root() {
    try {
        const auto share_dir =
            ament_index_cpp::get_package_share_directory("kuka_arctech_adapter");
        return share_dir + "/templates";
    } catch (const std::exception &) {
        return "templates";
    }
}

bool read_text_file(const std::filesystem::path &path, std::string &out) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        return false;
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    out = oss.str();
    return true;
}

std::string fmt_num(double v, int precision = 3) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << v;
    return oss.str();
}

std::string to_crlf(const std::string &text) {
    std::string out;
    out.reserve(text.size() + 64);
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\r') {
            continue;
        }
        if (text[i] == '\n') {
            out += "\r\n";
        } else {
            out.push_back(text[i]);
        }
    }
    return out;
}

bool has_arc_start_step(const KukaArcTechJobIR &job_ir) {
    for (const auto &step : job_ir.steps) {
        if (std::holds_alternative<ArcStartStep>(step.payload)) {
            return true;
        }
    }
    return false;
}

std::string first_wdat_ref(const KukaArcTechJobIR &job_ir) {
    for (const auto &step : job_ir.steps) {
        if (const auto *recipe = std::get_if<RecipeSelectStep>(&step.payload)) {
            if (!recipe->weld_data_ref.empty()) {
                return recipe->weld_data_ref;
            }
        }
    }
    return "";
}

// 将 joint_names（如 A1..A6 / joint_1..joint_6 / j1..j6）映射到 KRL 轴槽位 0..5
bool resolve_krl_axis_values(
    const MoveJointStep &payload,
    std::array<double, 6> &out_deg,
    std::string &error_message) {
    std::array<bool, 6> filled{false, false, false, false, false, false};
    for (std::size_t idx = 0; idx < 6; ++idx) {
        std::string name = payload.joint_names[idx];
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        int axis_slot = -1;
        if (name.size() == 2 && (name[0] == 'a' || name[0] == 'j') &&
            name[1] >= '1' && name[1] <= '6') {
            axis_slot = name[1] - '1';
        } else if (name.rfind("joint_", 0) == 0 && name.size() == 7 &&
                   name[6] >= '1' && name[6] <= '6') {
            axis_slot = name[6] - '1';
        } else if (name.rfind("joint", 0) == 0 && name.size() == 6 &&
                   name[5] >= '1' && name[5] <= '6') {
            axis_slot = name[5] - '1';
        }
        if (axis_slot < 0 || axis_slot >= 6) {
            error_message = "无法将关节名 '" + payload.joint_names[idx] +
                            "' 映射到 KUKA A1~A6（支持 A1..A6 / joint_1..joint_6 / j1..j6）。";
            return false;
        }
        if (filled[axis_slot]) {
            error_message = "joint_names 中存在重复映射到 A" + std::to_string(axis_slot + 1) + " 的关节名。";
            return false;
        }
        filled[axis_slot] = true;
        out_deg[axis_slot] = payload.target_rad[idx] * kRadToDeg;
    }
    return true;
}

// 严格白名单占位符替换：模板中若出现未在 allowed_placeholders 声明的 {{KEY}} 直接报错
ErrorCode render_placeholders(
    const std::string &tmpl,
    const std::unordered_set<std::string> &allowed,
    const std::unordered_map<std::string, std::string> &values,
    std::string &out,
    std::string &error_message) {
    out.clear();
    out.reserve(tmpl.size() + 512);

    std::size_t pos = 0;
    while (pos < tmpl.size()) {
        const std::size_t open = tmpl.find("{{", pos);
        if (open == std::string::npos) {
            out.append(tmpl, pos, std::string::npos);
            break;
        }
        out.append(tmpl, pos, open - pos);

        const std::size_t close = tmpl.find("}}", open + 2);
        if (close == std::string::npos) {
            error_message = "模板语法错误：存在未闭合的占位符 '{{'。";
            return ErrorCode::KA_TEMPLATE_RENDER_FAILED;
        }

        const std::string key = tmpl.substr(open + 2, close - (open + 2));
        if (allowed.find(key) == allowed.end()) {
            error_message = "模板包含未在 allowed_placeholders 白名单中声明的占位符: {{" + key + "}}。";
            return ErrorCode::KA_TEMPLATE_RENDER_FAILED;
        }

        const auto it = values.find(key);
        if (it == values.end()) {
            error_message = "渲染缺少白名单占位符的值: {{" + key + "}}。";
            return ErrorCode::KA_TEMPLATE_RENDER_FAILED;
        }

        out.append(it->second);
        pos = close + 2;
    }

    return ErrorCode::OK;
}

}  // namespace

TemplateEngine::TemplateEngine() : templates_root_(default_templates_root()) {}

TemplateEngine::TemplateEngine(const std::string &templates_root)
    : templates_root_(templates_root) {}

ErrorCode TemplateEngine::render(
    const KukaArcTechJobIR &job_ir,
    ProgramPackage &out_pkg,
    std::string &error_message) const {
    namespace fs = std::filesystem;

    // 根据是否空跑/是否含引弧选择对应黄金模板目录
    const bool is_real_weld = !job_ir.dry_run && has_arc_start_step(job_ir);
    const std::string rel_template_dir =
        is_real_weld ? "kss8_arctech38/straight_weld" : "kss8_arctech38/dry_run_motion";
    const fs::path tmpl_dir = fs::path(templates_root_) / rel_template_dir;

    const fs::path manifest_path = tmpl_dir / "template_manifest.json";
    const fs::path src_path = tmpl_dir / "golden.src";
    const fs::path dat_path = tmpl_dir / "golden.dat";

    if (!fs::exists(manifest_path) || !fs::exists(src_path) || !fs::exists(dat_path)) {
        error_message = "找不到黄金模板目录或必需文件: " + tmpl_dir.string();
        return ErrorCode::KA_TEMPLATE_NOT_FOUND;
    }

    std::string manifest_raw;
    std::string golden_src;
    std::string golden_dat;
    if (!read_text_file(manifest_path, manifest_raw) ||
        !read_text_file(src_path, golden_src) ||
        !read_text_file(dat_path, golden_dat)) {
        error_message = "读取黄金模板文件失败: " + tmpl_dir.string();
        return ErrorCode::KA_TEMPLATE_NOT_FOUND;
    }

    nlohmann::json manifest_doc;
    try {
        manifest_doc = nlohmann::json::parse(manifest_raw);
    } catch (const std::exception &e) {
        error_message = std::string("解析 template_manifest.json 失败: ") + e.what();
        return ErrorCode::KA_TEMPLATE_RENDER_FAILED;
    }

    if (!manifest_doc.is_object() ||
        !manifest_doc.contains("template_id") ||
        !manifest_doc.contains("allowed_placeholders") ||
        !manifest_doc.at("allowed_placeholders").is_array()) {
        error_message = "template_manifest.json 缺少 template_id 或 allowed_placeholders 数组。";
        return ErrorCode::KA_TEMPLATE_RENDER_FAILED;
    }

    std::unordered_set<std::string> allowed;
    for (const auto &entry : manifest_doc.at("allowed_placeholders")) {
        if (!entry.is_string()) {
            error_message = "template_manifest.json 的 allowed_placeholders 必须全为字符串。";
            return ErrorCode::KA_TEMPLATE_RENDER_FAILED;
        }
        allowed.insert(entry.get<std::string>());
    }

    std::vector<std::string> declared_wdats;
    if (manifest_doc.contains("declared_wdats") && manifest_doc.at("declared_wdats").is_array()) {
        for (const auto &entry : manifest_doc.at("declared_wdats")) {
            if (entry.is_string()) {
                declared_wdats.push_back(entry.get<std::string>());
            }
        }
    }

    const int tool_no = (job_ir.tool_id >= 0) ? job_ir.tool_id : 1;
    const int base_no = (job_ir.base_id >= 0) ? job_ir.base_id : 0;
    const std::string wdat_name = first_wdat_ref(job_ir);

    // 按真实 KSS 8.x (test1120) 风格生成各步运动指令与 .dat 声明
    std::ostringstream summary_oss;
    std::ostringstream src_body;
    std::ostringstream dat_body;

    summary_oss << "STEPS=" << job_ir.steps.size() << " [";
    int pt_idx = 0;
    int pdat_idx = 0;
    int ldat_idx = 0;

    bool render_step_failed = false;
    for (std::size_t i = 0; i < job_ir.steps.size(); ++i) {
        const auto &step = job_ir.steps[i];
        if (i > 0) {
            summary_oss << ", ";
        }
        summary_oss << step.index << ":" << step.instruction_id;

        src_body << "; --- Step " << step.index
                 << " (" << step.instruction_id
                 << " / " << step.instance_id << ") ---\n";

        std::visit(
            [&](const auto &payload) {
                using T = std::decay_t<decltype(payload)>;

                if constexpr (std::is_same_v<T, MoveJointStep>) {
                    std::array<double, 6> axis_deg{};
                    if (!resolve_krl_axis_values(payload, axis_deg, error_message)) {
                        render_step_failed = true;
                        return;
                    }
                    ++pt_idx;
                    ++pdat_idx;
                    const std::string j_name = "J" + std::to_string(pt_idx);
                    const std::string pdat_name = "PDAT" + std::to_string(pdat_idx);
                    // 将 rad/s 与 rad/s^2 映射到 [1, 100]% 关节速度与加速度百分比
                    const double vel_pct = std::clamp(
                        (payload.max_velocity_rad_s / 3.14) * 100.0, 1.0, 100.0);
                    const double acc_pct = std::clamp(
                        (payload.max_acceleration_rad_s2 / 10.0) * 100.0, 1.0, 100.0);

                    dat_body << "DECL E6AXIS X" << j_name << "={"
                             << "A1 " << fmt_num(axis_deg[0])
                             << ",A2 " << fmt_num(axis_deg[1])
                             << ",A3 " << fmt_num(axis_deg[2])
                             << ",A4 " << fmt_num(axis_deg[3])
                             << ",A5 " << fmt_num(axis_deg[4])
                             << ",A6 " << fmt_num(axis_deg[5])
                             << ",E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}\n";
                    dat_body << "DECL FDAT F" << j_name << "={TOOL_NO " << tool_no
                             << ",BASE_NO " << base_no
                             << ",IPO_FRAME #BASE,POINT2[] \" \"}\n";
                    dat_body << "DECL PDAT P" << pdat_name << "={VEL " << fmt_num(vel_pct)
                             << ",ACC " << fmt_num(acc_pct)
                             << ",APO_DIST 500.000,APO_MODE #CDIS,GEAR_JERK 100.000,EXAX_IGN 0}\n";

                    src_body << "SPTP X" << j_name
                             << " WITH $VEL_AXIS[1] = SVEL_JOINT(" << fmt_num(vel_pct, 1)
                             << "), $TOOL = STOOL2(F" << j_name
                             << "), $BASE = SBASE(F" << j_name << ".BASE_NO)"
                             << ", $IPO_MODE = SIPO_MODE(F" << j_name << ".IPO_FRAME)"
                             << ", $LOAD = SLOAD(F" << j_name << ".TOOL_NO)"
                             << ", $ACC_AXIS[1] = SACC_JOINT(P" << pdat_name
                             << "), $APO = SAPO_PTP(P" << pdat_name
                             << "), $GEAR_JERK[1] = SGEAR_JERK(P" << pdat_name
                             << "), $COLLMON_TOL_PRO[1] = USE_CM_PRO_VALUES(0)\n";
                } else if constexpr (std::is_same_v<T, MoveLinearStep>) {
                    ++pt_idx;
                    ++ldat_idx;
                    const std::string p_name = "P" + std::to_string(pt_idx);
                    const std::string ldat_name = "CPDAT" + std::to_string(ldat_idx);
                    const double vel = std::max(0.001, payload.speed_m_s);
                    const double acc_pct = std::clamp(
                        (payload.acceleration_m_s2 / 5.0) * 100.0, 1.0, 100.0);

                    dat_body << "DECL E6POS X" << p_name << "={"
                             << "X " << fmt_num(payload.target_pose.x)
                             << ",Y " << fmt_num(payload.target_pose.y)
                             << ",Z " << fmt_num(payload.target_pose.z)
                             << ",A " << fmt_num(payload.target_pose.a)
                             << ",B " << fmt_num(payload.target_pose.b)
                             << ",C " << fmt_num(payload.target_pose.c)
                             << ",E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}\n";
                    dat_body << "DECL FDAT F" << p_name << "={TOOL_NO " << tool_no
                             << ",BASE_NO " << base_no
                             << ",IPO_FRAME #BASE,POINT2[] \" \"}\n";
                    dat_body << "DECL LDAT L" << ldat_name << "={VEL " << fmt_num(vel, 4)
                             << ",ACC " << fmt_num(acc_pct)
                             << ",APO_DIST 500.000,APO_FAC 50.0000,AXIS_VEL 100.000,"
                             << "AXIS_ACC " << fmt_num(acc_pct)
                             << ",ORI_TYP #VAR,CIRC_TYP #BASE,JERK_FAC 50.0000,"
                             << "GEAR_JERK 100.000,EXAX_IGN 0}\n";

                    src_body << "SLIN X" << p_name
                             << " WITH $VEL = SVEL_CP(" << fmt_num(vel, 4) << ", , L" << ldat_name
                             << "), $TOOL = STOOL2(F" << p_name
                             << "), $BASE = SBASE(F" << p_name << ".BASE_NO)"
                             << ", $IPO_MODE = SIPO_MODE(F" << p_name << ".IPO_FRAME)"
                             << ", $LOAD = SLOAD(F" << p_name << ".TOOL_NO)"
                             << ", $ACC = SACC_CP(L" << ldat_name
                             << "), $ORI_TYPE = SORI_TYP(L" << ldat_name
                             << "), $APO = SAPO(L" << ldat_name
                             << "), $JERK = SJERK(L" << ldat_name
                             << "), $COLLMON_TOL_PRO[1] = USE_CM_PRO_VALUES(0)\n";
                } else if constexpr (std::is_same_v<T, FollowPathStep>) {
                    ++ldat_idx;
                    const std::string ldat_name = "CPDAT" + std::to_string(ldat_idx);
                    const double vel = std::max(0.001, payload.speed_m_s);

                    dat_body << "DECL LDAT L" << ldat_name << "={VEL " << fmt_num(vel, 4)
                             << ",ACC 100.000,APO_DIST 500.000,APO_FAC 50.0000,AXIS_VEL 100.000,"
                             << "AXIS_ACC 100.000,ORI_TYP #VAR,CIRC_TYP #BASE,JERK_FAC 50.0000,"
                             << "GEAR_JERK 100.000,EXAX_IGN 0}\n";

                    for (const auto &wp : payload.waypoints) {
                        ++pt_idx;
                        const std::string p_name = "P" + std::to_string(pt_idx);
                        dat_body << "DECL E6POS X" << p_name << "={"
                                 << "X " << fmt_num(wp.x)
                                 << ",Y " << fmt_num(wp.y)
                                 << ",Z " << fmt_num(wp.z)
                                 << ",A " << fmt_num(wp.a)
                                 << ",B " << fmt_num(wp.b)
                                 << ",C " << fmt_num(wp.c)
                                 << ",E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}\n";
                        dat_body << "DECL FDAT F" << p_name << "={TOOL_NO " << tool_no
                                 << ",BASE_NO " << base_no
                                 << ",IPO_FRAME #BASE,POINT2[] \" \"}\n";

                        src_body << "SLIN X" << p_name
                                 << " WITH $VEL = SVEL_CP(" << fmt_num(vel, 4) << ", , L" << ldat_name
                                 << "), $TOOL = STOOL2(F" << p_name
                                 << "), $BASE = SBASE(F" << p_name << ".BASE_NO)"
                                 << ", $IPO_MODE = SIPO_MODE(F" << p_name << ".IPO_FRAME)"
                                 << ", $LOAD = SLOAD(F" << p_name << ".TOOL_NO)"
                                 << ", $ACC = SACC_CP(L" << ldat_name
                                 << "), $ORI_TYPE = SORI_TYP(L" << ldat_name
                                 << "), $APO = SAPO(L" << ldat_name
                                 << "), $JERK = SJERK(L" << ldat_name
                                 << "), $COLLMON_TOL_PRO[1] = USE_CM_PRO_VALUES(0)\n";
                    }
                } else if constexpr (std::is_same_v<T, RecipeSelectStep>) {
                    src_body << "; WLD-003 SelectWeldProgram program_id=" << payload.program_id
                             << " -> WDAT=" << payload.weld_data_ref << "\n";
                } else if constexpr (std::is_same_v<T, ArcStartStep>) {
                    if (is_real_weld) {
                        src_body << "; [UNVERIFIED_ARCTECH_PLACEHOLDER] ARC_ON"
                                 << " profile=" << payload.start_profile_id
                                 << " wdat=" << wdat_name
                                 << " seam_id=" << job_ir.seam_id
                                 << " lease_ms=" << payload.lease_ms << "\n";
                    } else {
                        src_body << "; DRY_RUN: arc ignition suppressed (WLD-015 profile="
                                 << payload.start_profile_id << ")\n";
                    }
                } else if constexpr (std::is_same_v<T, ArcStopStep>) {
                    if (is_real_weld) {
                        src_body << "; [UNVERIFIED_ARCTECH_PLACEHOLDER] ARC_OFF"
                                 << " crater_profile=" << payload.crater_profile_id << "\n";
                    } else {
                        src_body << "; DRY_RUN: arc stop suppressed (WLD-016 crater="
                                 << payload.crater_profile_id << ")\n";
                    }
                } else if constexpr (std::is_same_v<T, MeasureProcessStep>) {
                    src_body << "; WLD-024 GetWeldMeasurements channels=";
                    for (std::size_t c = 0; c < payload.channels.size(); ++c) {
                        if (c > 0) {
                            src_body << ",";
                        }
                        src_body << payload.channels[c];
                    }
                    src_body << "\n";
                }
            },
            step.payload);
        if (render_step_failed) {
            return ErrorCode::KA_TEMPLATE_RENDER_FAILED;
        }
    }
    summary_oss << "]";

    const std::unordered_map<std::string, std::string> values{
        {"PROGRAM_NAME", job_ir.program_name},
        {"JOB_ID", job_ir.job_id},
        {"TRACE_ID", job_ir.trace_id},
        {"TOOL_ID", std::to_string(tool_no)},
        {"BASE_ID", std::to_string(base_no)},
        {"WDAT_NAME", wdat_name.empty() ? "NONE" : wdat_name},
        {"SEAM_ID", std::to_string(job_ir.seam_id)},
        {"STEP_SUMMARY", summary_oss.str()},
        {"MOTION_STATEMENTS", src_body.str()},
        {"DATA_DECLS", dat_body.str()},
    };

    std::string rendered_src;
    if (const auto rc =
            render_placeholders(golden_src, allowed, values, rendered_src, error_message);
        rc != ErrorCode::OK) {
        return rc;
    }

    std::string rendered_dat;
    if (const auto rc =
            render_placeholders(golden_dat, allowed, values, rendered_dat, error_message);
        rc != ErrorCode::OK) {
        return rc;
    }

    out_pkg.program_name = job_ir.program_name;
    out_pkg.template_id = manifest_doc.at("template_id").get<std::string>();
    out_pkg.template_verified_on_krc = manifest_doc.value("verified_on_krc", false);
    out_pkg.dry_run = job_ir.dry_run;
    out_pkg.wdat_ref = wdat_name;
    out_pkg.declared_wdats = std::move(declared_wdats);
    out_pkg.src_content = to_crlf(rendered_src);
    out_pkg.dat_content = to_crlf(rendered_dat);
    out_pkg.manifest_json = "{}";
    return ErrorCode::OK;
}
