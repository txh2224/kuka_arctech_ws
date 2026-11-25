#ifndef MANUAL_KRC_TRANSPORT_H
#define MANUAL_KRC_TRANSPORT_H

#include <mutex>
#include <string>
#include <vector>

#include "krc_transport.h"

// 规格书第 9 节：人工导入兜底实现（将 .src / .dat / artifact_manifest.json 落盘，等待现场操作员导入 KRC）
class ManualKrcTransport : public IKrcTransport {
public:
    explicit ManualKrcTransport(std::string export_root = "output");

    std::string mode_name() const override { return "MANUAL"; }

    ProbeResult probe() override;
    DeployResult deploy(const ProgramPackage &pkg) override;
    CommandResult select_program(const std::string &name) override;
    CommandResult start_program(const std::string &name) override;
    CommandResult stop_program() override;
    KrcStatus read_status() override;
    SeamHandshakeResult set_seam_permission(uint32_t seam_id, bool allow) override;
    std::vector<KrcFault> read_faults() override;
    void set_runtime_state(
        const std::string &program_state,
        const std::string &weld_state,
        const std::string &message) override;

    std::string last_export_dir() const;

private:
    mutable std::mutex mutex_;
    std::string export_root_;
    std::string last_export_dir_;
    KrcStatus status_;
};

#endif  // MANUAL_KRC_TRANSPORT_H
