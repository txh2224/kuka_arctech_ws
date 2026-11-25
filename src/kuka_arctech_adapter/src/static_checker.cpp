#include "static_checker.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iomanip>
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

std::size_t count_substring(const std::string &text, const std::string &needle) {
    std::size_t count = 0;
    std::size_t pos = 0;
    while ((pos = text.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    return count;
}

inline uint32_t rotr32(uint32_t x, uint32_t n) {
    return (x >> n) | (x << (32 - n));
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
    static constexpr std::array<uint32_t, 64> k = {
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
        0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
        0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
        0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
        0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
        0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
        0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
        0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
        0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
        0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
        0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
        0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
        0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
        0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
        0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
        0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
    };

    std::array<uint32_t, 8> h = {
        0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
        0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u,
    };

    std::vector<uint8_t> msg(data.begin(), data.end());
    const uint64_t bit_len = static_cast<uint64_t>(msg.size()) * 8u;
    msg.push_back(0x80u);
    while ((msg.size() % 64u) != 56u) {
        msg.push_back(0x00u);
    }
    for (int i = 7; i >= 0; --i) {
        msg.push_back(static_cast<uint8_t>((bit_len >> (i * 8)) & 0xffu));
    }

    for (std::size_t chunk = 0; chunk < msg.size(); chunk += 64u) {
        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            const std::size_t idx = chunk + static_cast<std::size_t>(i) * 4u;
            w[i] = (static_cast<uint32_t>(msg[idx]) << 24) |
                   (static_cast<uint32_t>(msg[idx + 1]) << 16) |
                   (static_cast<uint32_t>(msg[idx + 2]) << 8) |
                   (static_cast<uint32_t>(msg[idx + 3]));
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
        uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];

        for (int i = 0; i < 64; ++i) {
            const uint32_t S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
            const uint32_t ch = (e & f) ^ ((~e) & g);
            const uint32_t temp1 = hh + S1 + ch + k[i] + w[i];
            const uint32_t S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t temp2 = S0 + maj;

            hh = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }

        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
        h[5] += f;
        h[6] += g;
        h[7] += hh;
    }

    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (uint32_t word : h) {
        oss << std::setw(8) << word;
    }
    return oss.str();
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

    // 4. 引弧 / 收弧成对与 dry_run 防误引弧检查
    const std::size_t arc_on_count =
        count_substring(pkg.src_content, "ARC_ON") + count_substring(pkg.src_content, "ARCON");
    const std::size_t arc_off_count =
        count_substring(pkg.src_content, "ARC_OFF") + count_substring(pkg.src_content, "ARCOFF");
    if (arc_on_count != arc_off_count) {
        error_message = "静态检查失败：.src 中引弧(ARC_ON/ARCON)与收弧(ARC_OFF/ARCOFF)不成对。";
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

    // 7. 生成 artifact_manifest.json（含 SHA-256）
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
