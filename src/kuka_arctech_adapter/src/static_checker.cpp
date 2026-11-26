#include "static_checker.h"
#include "sha256.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

namespace {

std::string default_validator_config_path() {
    try {
        const auto share_dir =
            ament_index_cpp::get_package_share_directory("kuka_arctech_adapter");
        return share_dir + "/config/validator.json";
    } catch (const std::exception &) {
        return "config/validator.json";
    }
}

bool is_valid_krl_program_name(const std::string &name, std::size_t max_len) {
    if (name.empty() || name.size() > max_len) {
        return false;
    }
    if (!std::isalpha(static_cast<unsigned char>(name.front()))) {
        return false;
    }
    return std::all_of(name.begin(), name.end(), [](unsigned char c) {
        return std::isalnum(c) || c == '_';
    });
}

std::string trim_line(const std::string &s) {
    std::size_t b = 0;
    while (b < s.size() && (s[b] == ' ' || s[b] == '\t' || s[b] == '\r' || s[b] == '\n')) {
        ++b;
    }
    std::size_t e = s.size();
    while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r' || s[e - 1] == '\n')) {
        --e;
    }
    return s.substr(b, e - b);
}

}  // namespace

StaticChecker::StaticChecker() {
    load_config(default_validator_config_path());
}

StaticChecker::StaticChecker(const std::string &validator_config_path) {
    load_config(validator_config_path);
}

void StaticChecker::load_config(const std::string &config_path) {
    std::ifstream ifs(config_path);
    if (!ifs.is_open()) {
        return;
    }
    try {
        nlohmann::json doc;
        ifs >> doc;
        if (doc.contains("max_program_name_length") &&
            doc.at("max_program_name_length").is_number_unsigned()) {
            max_program_name_length_ = doc.at("max_program_name_length").get<std::size_t>();
        }
        if (doc.contains("wdat_allowlist") && doc.at("wdat_allowlist").is_array()) {
            wdat_allowlist_.clear();
            for (const auto &item : doc.at("wdat_allowlist")) {
                if (item.is_string()) {
                    wdat_allowlist_.insert(item.get<std::string>());
                }
            }
        }
    } catch (const std::exception &) {
        // 保留默认值
    }
}

std::string StaticChecker::sha256_hex(const std::string &data) {
    return utils::sha256_hex(data);
}

ErrorCode StaticChecker::check_and_build_manifest(
    const KukaArcTechJobIR &job_ir,
    const std::string &transport_mode,
    ProgramPackage &pkg,
    std::string &error_message) const {
    // 1. 程序名规范检查
    if (!is_valid_krl_program_name(pkg.program_name, max_program_name_length_)) {
        error_message = "静态检查失败：KRL 程序名 '" + pkg.program_name + "' 不合法。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }

    // 2. SRC / DAT 基础结构检查
    if (pkg.src_content.empty() || pkg.dat_content.empty()) {
        error_message = "静态检查失败：生成的 .src 或 .dat 内容为空。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }

    if (pkg.src_content.find("DEF " + pkg.program_name) == std::string::npos) {
        error_message = "静态检查失败：.src 缺少匹配的 DEF " + pkg.program_name + " 定义。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }
    if (pkg.dat_content.find("DEFDAT " + pkg.program_name) == std::string::npos) {
        error_message = "静态检查失败：.dat 缺少匹配的 DEFDAT " + pkg.program_name + " 定义。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }

    // 检查尾行 END / ENDDAT
    auto last_non_empty_line = [](const std::string &content) -> std::string {
        std::istringstream iss(content);
        std::string line;
        std::string last;
        while (std::getline(iss, line)) {
            const std::string t = trim_line(line);
            if (!t.empty()) {
                last = t;
            }
        }
        return last;
    };

    if (last_non_empty_line(pkg.src_content) != "END") {
        error_message = "静态检查失败：.src 未以 END 结束。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }
    if (last_non_empty_line(pkg.dat_content) != "ENDDAT") {
        error_message = "静态检查失败：.dat 未以 ENDDAT 结束。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }

    // 3. 禁止残留未替换占位符
    if (pkg.src_content.find("{{") != std::string::npos ||
        pkg.src_content.find("}}") != std::string::npos ||
        pkg.dat_content.find("{{") != std::string::npos ||
        pkg.dat_content.find("}}") != std::string::npos) {
        error_message = "静态检查失败：生成产物中存在未替换的模板占位符（{{ / }}）。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }

    // 4. 非注释代码行中的引弧 / 收弧成对与顺序检查（以及 dry_run 防误引弧）
    int arc_depth = 0;
    std::size_t arc_on_count = 0;
    std::size_t arc_off_count = 0;
    {
        std::istringstream iss(pkg.src_content);
        std::string raw_line;
        while (std::getline(iss, raw_line)) {
            const std::size_t semi = raw_line.find(';');
            std::string code = trim_line(semi == std::string::npos ? raw_line : raw_line.substr(0, semi));
            if (code.empty()) {
                continue;
            }
            const bool is_on =
                code.find("ARC_ON") != std::string::npos || code.find("ARCON") != std::string::npos;
            const bool is_off =
                code.find("ARC_OFF") != std::string::npos || code.find("ARCOFF") != std::string::npos;
            if (is_on) {
                ++arc_on_count;
                ++arc_depth;
            }
            if (is_off) {
                ++arc_off_count;
                --arc_depth;
                if (arc_depth < 0) {
                    error_message = "静态检查失败：.src 中出现未先引弧的收弧指令（ARC_OFF/ARCOFF 顺序错误）。";
                    return ErrorCode::KA_STATIC_CHECK_FAILED;
                }
            }
        }
    }
    if (arc_on_count != arc_off_count || arc_depth != 0) {
        error_message = "静态检查失败：.src 中非注释引弧(ARC_ON/ARCON)与收弧(ARC_OFF/ARCOFF)不成对。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }
    if (job_ir.dry_run && arc_on_count > 0) {
        error_message = "静态检查失败：dry_run=true 时禁止在 .src 中包含引弧指令。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }

    // 5. WDAT 白名单与模板 manifest 一致性检查
    if (!pkg.wdat_ref.empty()) {
        if (wdat_allowlist_.find(pkg.wdat_ref) == wdat_allowlist_.end()) {
            error_message = "静态检查失败：WDAT '" + pkg.wdat_ref + "' 不在 validator.json 白名单中。";
            return ErrorCode::KA_STATIC_CHECK_FAILED;
        }
        if (std::find(pkg.declared_wdats.begin(), pkg.declared_wdats.end(), pkg.wdat_ref) ==
            pkg.declared_wdats.end()) {
            error_message = "静态检查失败：WDAT '" + pkg.wdat_ref + "' 未在模板 manifest 中声明。";
            return ErrorCode::KA_STATIC_CHECK_FAILED;
        }
    }

    // 6. 未在真机验证过的占位模板禁止在非 MOCK 模式下导出或部署
    if (transport_mode != "MOCK" && !pkg.template_verified_on_krc) {
        error_message = "静态检查失败：模板 '" + pkg.template_id +
                        "' 尚未在真实 KRC 示教器上验证（verified_on_krc=false），禁止在 " +
                        transport_mode + " 模式下导出或部署。";
        return ErrorCode::KA_STATIC_CHECK_FAILED;
    }

    // 7. 产物清单将这次作业、模板与生成文件的摘要关联起来。
    // 摘要覆盖渲染后的完整 SRC/DAT 字节（含 CRLF 换行），供导出或传输后核对。
    const std::string src_hash = sha256_hex(pkg.src_content);
    const std::string dat_hash = sha256_hex(pkg.dat_content);

    nlohmann::json manifest{
        {"job_id", job_ir.job_id},
        {"trace_id", job_ir.trace_id},
        {"backend", "kuka_arctech"},
        {"template_id", pkg.template_id},
        {"template_verified_on_krc", pkg.template_verified_on_krc},
        {"program_name", pkg.program_name},
        {"src_sha256", src_hash},
        {"dat_sha256", dat_hash},
        {"wdat_ref", pkg.wdat_ref},
        {"dry_run", job_ir.dry_run},
    };
    pkg.manifest_json = manifest.dump();
    return ErrorCode::OK;
}
