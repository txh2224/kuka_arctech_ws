#include "manual_krc_transport.h"

#include <filesystem>
#include <fstream>

ManualKrcTransport::ManualKrcTransport(std::string export_root)
    : export_root_(std::move(export_root)) {
    // Manual 模式仅做本地程序包导出，未连接真实 KRC 控制器
    status_.controller_connected = false;
    status_.program_state = "IDLE";
    status_.weld_state = "ARC_OFF";
    status_.message = "Manual offline export transport ready (controller_connected=false, export_root=" + export_root_ + ").";
}

ProbeResult ManualKrcTransport::probe() {
    namespace fs = std::filesystem;
    std::lock_guard<std::mutex> lock(mutex_);
    ProbeResult res;
    std::error_code ec;
    fs::create_directories(export_root_, ec);
    status_.controller_connected = false;
    res.connected = false;
    if (ec) {
        status_.program_state = "FAULT";
        status_.fault_code = to_error_code_string(ErrorCode::KA_DEPLOY_FAILED);
        status_.message = "无法创建 Manual 导出目录 " + export_root_ + ": " + ec.message();
        res.error_code = ErrorCode::KA_DEPLOY_FAILED;
        res.message = status_.message;
        return res;
    }

    status_.program_state = "READY";
    status_.fault_code.clear();
    status_.message = "Manual 离线导出目录就绪（未连接 KRC）: " + export_root_;
    res.error_code = ErrorCode::OK;
    res.message = status_.message;
    return res;
}

DeployResult ManualKrcTransport::deploy(const ProgramPackage &pkg) {
    namespace fs = std::filesystem;
    std::lock_guard<std::mutex> lock(mutex_);
    DeployResult res;

    if (pkg.program_name.empty()) {
        res.ok = false;
        res.error_code = ErrorCode::KA_DEPLOY_FAILED;
        res.message = "导出失败：程序名为空。";
        return res;
    }

    std::error_code ec;
    const fs::path abs_target = fs::absolute(fs::path(export_root_) / pkg.program_name, ec);
    const fs::path target_dir = ec ? (fs::path(export_root_) / pkg.program_name) : abs_target.lexically_normal();
    ec.clear();
    fs::create_directories(target_dir, ec);
    if (ec) {
        res.ok = false;
        res.error_code = ErrorCode::KA_DEPLOY_FAILED;
        res.message = "创建程序导出目录失败 (" + target_dir.string() + "): " + ec.message();
        res.raw_evidence = ec.message();
        return res;
    }

    auto write_file_atomic = [&](const fs::path &p, const std::string &content) -> bool {
        const fs::path tmp = p.string() + ".tmp";
        {
            std::ofstream ofs(tmp, std::ios::binary | std::ios::trunc);
            if (!ofs.is_open()) {
                return false;
            }
            ofs.write(content.data(), static_cast<std::streamsize>(content.size()));
            ofs.flush();
            if (!ofs.good()) {
                return false;
            }
        }
        std::error_code ren_ec;
        fs::rename(tmp, p, ren_ec);
        if (ren_ec) {
            std::error_code rm_ec;
            fs::remove(tmp, rm_ec);
            return false;
        }
        return true;
    };

    const fs::path src_file = target_dir / (pkg.program_name + ".src");
    const fs::path dat_file = target_dir / (pkg.program_name + ".dat");
    const fs::path manifest_file = target_dir / "artifact_manifest.json";

    if (!write_file_atomic(src_file, pkg.src_content) ||
        !write_file_atomic(dat_file, pkg.dat_content) ||
        !write_file_atomic(manifest_file, pkg.manifest_json)) {
        res.ok = false;
        res.error_code = ErrorCode::KA_DEPLOY_FAILED;
        res.message = "写入 .src/.dat/artifact_manifest.json 失败: " + target_dir.string();
        res.raw_evidence = target_dir.string();
        return res;
    }

    last_export_dir_ = target_dir.string();
    status_.current_program = pkg.program_name;
    status_.program_state = "WAITING_OPERATOR";
    status_.weld_state = "ARC_OFF";
    status_.fault_code.clear();
    status_.message = "程序包已导出至 " + last_export_dir_ + "，请人工导入 KRC 执行。";

    res.ok = true;
    res.error_code = ErrorCode::OK;
    res.message = status_.message;
    res.raw_evidence = last_export_dir_;
    return res;
}

CommandResult ManualKrcTransport::select_program(const std::string &name) {
    std::lock_guard<std::mutex> lock(mutex_);
    CommandResult res;
    status_.current_program = name;
    status_.program_state = "WAITING_OPERATOR";
    res.ok = true;
    res.error_code = ErrorCode::OK;
    res.message = "Manual 模式：请操作员在 smartHMI 上选定程序 " + name + "。";
    return res;
}

CommandResult ManualKrcTransport::start_program(const std::string &name) {
    std::lock_guard<std::mutex> lock(mutex_);
    CommandResult res;
    status_.current_program = name;
    status_.program_state = "WAITING_OPERATOR";
    res.ok = true;
    res.error_code = ErrorCode::OK;
    res.message = "Manual 模式：已导出程序 " + name + "，等待操作员确认启动。";
    return res;
}

CommandResult ManualKrcTransport::stop_program() {
    std::lock_guard<std::mutex> lock(mutex_);
    CommandResult res;
    status_.program_state = "STOPPED";
    status_.weld_state = "ARC_OFF";
    status_.message = "Manual 模式任务已停止。";
    res.ok = true;
    res.error_code = ErrorCode::OK;
    res.message = status_.message;
    return res;
}

KrcStatus ManualKrcTransport::read_status() {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

SeamHandshakeResult ManualKrcTransport::set_seam_permission(uint32_t seam_id, bool) {
    SeamHandshakeResult res;
    res.requested_seam_id = seam_id;
    res.granted_seam_id = 0;
    res.granted = false;
    res.error_code = ErrorCode::KA_SEAM_PERMISSION_TIMEOUT;
    res.message = "Manual 模式不执行在线自动焊缝许可握手。";
    return res;
}

std::vector<KrcFault> ManualKrcTransport::read_faults() {
    return {};
}

void ManualKrcTransport::set_runtime_state(
    const std::string &program_state,
    const std::string &weld_state,
    const std::string &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.program_state = program_state;
    status_.weld_state = weld_state;
    status_.message = message;
}

std::string ManualKrcTransport::last_export_dir() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_export_dir_;
}
