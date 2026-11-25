#ifndef STATIC_CHECKER_H
#define STATIC_CHECKER_H

#include <string>
#include <unordered_set>

#include "error_code.h"
#include "job_ir.h"
#include "krc_transport.h"

// 规格书第 8 节：KRL 程序包静态校验 + SHA-256 摘要 + artifact_manifest.json 生成
class StaticChecker {
public:
    StaticChecker();
    explicit StaticChecker(const std::string &validator_config_path);

    // 校验 ProgramPackage 并填充 pkg.manifest_json；未通过返回 KA_STATIC_CHECK_FAILED
    ErrorCode check_and_build_manifest(
        const KukaArcTechJobIR &job_ir,
        const std::string &transport_mode,
        ProgramPackage &pkg,
        std::string &error_message) const;

    static std::string sha256_hex(const std::string &data);

private:
    void load_config(const std::string &config_path);

    std::size_t max_program_name_length_{24};
    std::unordered_set<std::string> wdat_allowlist_{"WDAT_DEMO_01"};
};

#endif  // STATIC_CHECKER_H
