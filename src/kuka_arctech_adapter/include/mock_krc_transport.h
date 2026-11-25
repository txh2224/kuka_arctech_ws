#ifndef MOCK_KRC_TRANSPORT_H
#define MOCK_KRC_TRANSPORT_H

#include <mutex>
#include <string>
#include <vector>

#include "krc_transport.h"

// 故障/异常注入配置（方便后续自动化测试 KA-T06 ~ KA-T09）
struct MockTransportOptions {
    bool reachable{true};
    bool fail_deploy{false};
    bool fail_start{false};
    bool fail_stop{false};
    bool deny_seam_permission{false};
    std::string injected_fault_code;
    std::string injected_fault_message;
};

// 规格书第 9 节：用于无机器人联调和全自动 CI 的 Mock KRC Transport
class MockKrcTransport : public IKrcTransport {
public:
    explicit MockKrcTransport(MockTransportOptions options = {});
    ~MockKrcTransport() override = default;

    std::string mode_name() const override { return "MOCK"; }

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

    void set_options(const MockTransportOptions &options);

private:
    mutable std::mutex mutex_;
    MockTransportOptions options_;
    KrcStatus status_;
    std::vector<KrcFault> faults_;
    std::string deployed_program_;
};

#endif  // MOCK_KRC_TRANSPORT_H
