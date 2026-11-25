#ifndef KRC_TRANSPORT_H
#define KRC_TRANSPORT_H

#include <cstdint>
#include <string>
#include <vector>

#include "error_code.h"

// 模板引擎生成的程序包结构（路线 B 填充）
struct ProgramPackage {
    std::string program_name;
    std::string template_id;
    bool template_verified_on_krc{false};
    bool dry_run{true};
    std::string wdat_ref;
    std::vector<std::string> declared_wdats;
    std::string src_content;
    std::string dat_content;
    std::string manifest_json{"{}"};
};

struct ProbeResult {
    bool connected{false};
    ErrorCode error_code{ErrorCode::OK};
    std::string message;
};

struct DeployResult {
    bool ok{false};
    ErrorCode error_code{ErrorCode::OK};
    std::string message;
    std::string raw_evidence;
};

struct CommandResult {
    bool ok{false};
    ErrorCode error_code{ErrorCode::OK};
    std::string message;
    std::string raw_evidence;
};

struct SeamHandshakeResult {
    bool granted{false};
    uint32_t requested_seam_id{0};
    uint32_t granted_seam_id{0};
    ErrorCode error_code{ErrorCode::OK};
    std::string message;
};

struct KrcFault {
    std::string code;
    std::string message;
};

struct KrcStatus {
    bool controller_connected{false};
    std::string program_state{"IDLE"};  // IDLE / READY / RUNNING / STOPPED / FAULT
    std::string weld_state{"ARC_OFF"};  // ARC_OFF / ARC_ON / WELDING / FAULT
    std::string current_program;
    std::string fault_code;
    std::string message;
};

// 规格书第 9 节：KRC 交互抽象接口（Mock / Manual / Verified 统一实现此接口）
class IKrcTransport {
public:
    virtual ~IKrcTransport() = default;

    virtual std::string mode_name() const = 0;

    virtual ProbeResult probe() = 0;
    virtual DeployResult deploy(const ProgramPackage &pkg) = 0;
    virtual CommandResult select_program(const std::string &name) = 0;
    virtual CommandResult start_program(const std::string &name) = 0;
    virtual CommandResult stop_program() = 0;
    virtual KrcStatus read_status() = 0;
    virtual SeamHandshakeResult set_seam_permission(uint32_t seam_id, bool allow) = 0;
    virtual std::vector<KrcFault> read_faults() = 0;

    // 供执行器在 Mock/状态机推进时同步控制器状态（真实控制器可为空操作，由底层轮询更新）
    virtual void set_runtime_state(
        const std::string &program_state,
        const std::string &weld_state,
        const std::string &message) = 0;
};

#endif  // KRC_TRANSPORT_H
