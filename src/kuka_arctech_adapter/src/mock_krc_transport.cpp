#include "mock_krc_transport.h"

MockKrcTransport::MockKrcTransport(MockTransportOptions options)
    : options_(std::move(options)) {
    status_.controller_connected = options_.reachable;
    status_.program_state = "IDLE";
    status_.weld_state = "ARC_OFF";
    status_.message = "Mock KRC ready";
}

void MockKrcTransport::set_options(const MockTransportOptions &options) {
    std::lock_guard<std::mutex> lock(mutex_);
    options_ = options;
    status_.controller_connected = options_.reachable;
    if (!options_.injected_fault_code.empty()) {
        faults_ = {{options_.injected_fault_code, options_.injected_fault_message}};
        status_.fault_code = options_.injected_fault_code;
    } else {
        faults_.clear();
        status_.fault_code.clear();
    }
}

ProbeResult MockKrcTransport::probe() {
    std::lock_guard<std::mutex> lock(mutex_);
    ProbeResult res;
    if (!options_.reachable) {
        status_.controller_connected = false;
        res.connected = false;
        res.error_code = ErrorCode::KA_KRC_UNREACHABLE;
        res.message = "Mock KRC 控制器不可达。";
        return res;
    }
    status_.controller_connected = true;
    res.connected = true;
    res.error_code = ErrorCode::OK;
    res.message = "Mock KRC 探活成功。";
    return res;
}

DeployResult MockKrcTransport::deploy(const ProgramPackage &pkg) {
    std::lock_guard<std::mutex> lock(mutex_);
    DeployResult res;
    if (!options_.reachable) {
        res.ok = false;
        res.error_code = ErrorCode::KA_KRC_UNREACHABLE;
        res.message = "部署失败：Mock KRC 控制器不可达。";
        return res;
    }
    if (options_.fail_deploy) {
        res.ok = false;
        res.error_code = ErrorCode::KA_DEPLOY_FAILED;
        res.message = "Mock 注入部署失败。";
        res.raw_evidence = "MOCK_DEPLOY_REJECTED";
        return res;
    }
    deployed_program_ = pkg.program_name;
    status_.current_program = pkg.program_name;
    status_.program_state = "READY";
    status_.message = "Program deployed: " + pkg.program_name;

    res.ok = true;
    res.error_code = ErrorCode::OK;
    res.message = "Mock 程序包部署成功。";
    return res;
}

CommandResult MockKrcTransport::select_program(const std::string &name) {
    std::lock_guard<std::mutex> lock(mutex_);
    CommandResult res;
    if (!options_.reachable) {
        res.ok = false;
        res.error_code = ErrorCode::KA_KRC_UNREACHABLE;
        res.message = "选程序失败：Mock KRC 控制器不可达。";
        return res;
    }
    status_.current_program = name;
    status_.program_state = "READY";
    status_.message = "Program selected: " + name;

    res.ok = true;
    res.error_code = ErrorCode::OK;
    res.message = "Mock 选程序成功。";
    return res;
}

CommandResult MockKrcTransport::start_program(const std::string &name) {
    std::lock_guard<std::mutex> lock(mutex_);
    CommandResult res;
    if (!options_.reachable) {
        res.ok = false;
        res.error_code = ErrorCode::KA_KRC_UNREACHABLE;
        res.message = "启动失败：Mock KRC 控制器不可达。";
        return res;
    }
    if (options_.fail_start) {
        status_.program_state = "FAULT";
        res.ok = false;
        res.error_code = ErrorCode::KA_PROGRAM_START_FAILED;
        res.message = "Mock 注入程序启动失败。";
        res.raw_evidence = "MOCK_START_REJECTED";
        return res;
    }
    status_.current_program = name;
    status_.program_state = "RUNNING";
    status_.message = "Program running: " + name;

    res.ok = true;
    res.error_code = ErrorCode::OK;
    res.message = "Mock 程序启动成功。";
    return res;
}

CommandResult MockKrcTransport::stop_program() {
    std::lock_guard<std::mutex> lock(mutex_);
    CommandResult res;
    if (!options_.reachable) {
        status_.controller_connected = false;
        res.ok = false;
        res.error_code = ErrorCode::KA_KRC_UNREACHABLE;
        res.message = "停止失败：Mock KRC 控制器不可达。";
        return res;
    }
    if (options_.fail_stop) {
        res.ok = false;
        res.error_code = ErrorCode::KA_STOP_FAILED;
        res.message = "Mock 注入停止失败。";
        res.raw_evidence = "MOCK_STOP_FAILED";
        return res;
    }
    status_.program_state = "STOPPED";
    status_.weld_state = "ARC_OFF";
    status_.message = "Program stopped";

    res.ok = true;
    res.error_code = ErrorCode::OK;
    res.message = "Mock 程序已停止。";
    return res;
}

KrcStatus MockKrcTransport::read_status() {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

SeamHandshakeResult MockKrcTransport::set_seam_permission(uint32_t seam_id, bool allow) {
    std::lock_guard<std::mutex> lock(mutex_);
    SeamHandshakeResult res;
    res.requested_seam_id = seam_id;
    if (!allow || options_.deny_seam_permission) {
        res.granted = false;
        res.granted_seam_id = 0;
        res.error_code = ErrorCode::KA_SEAM_PERMISSION_TIMEOUT;
        res.message = "Mock 焊缝许可未授予（seam_id=" + std::to_string(seam_id) + "）。";
        return res;
    }
    res.granted = true;
    res.granted_seam_id = seam_id;
    res.error_code = ErrorCode::OK;
    res.message = "Mock 焊缝许可已授予。";
    return res;
}

std::vector<KrcFault> MockKrcTransport::read_faults() {
    std::lock_guard<std::mutex> lock(mutex_);
    return faults_;
}

void MockKrcTransport::set_runtime_state(
    const std::string &program_state,
    const std::string &weld_state,
    const std::string &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.program_state = program_state;
    status_.weld_state = weld_state;
    status_.message = message;
}
