#ifndef SEMANTIC_MAPPER_H
#define SEMANTIC_MAPPER_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

#include <atomic_adapter_interfaces/action/execute_backend_job.hpp>

#include "error_code.h"
#include "job_ir.h"

// A组业务名称到KUKA内部编号/WDAT的映射配置（由 config/mapper.json 加载）
struct MapperConfig {
    std::string program_name_prefix{"AI_"};
    std::size_t max_program_name_length{24};
    uint32_t default_seam_id{1};

    std::unordered_map<std::string, int> tool_id_map{{"welding_torch_01", 1}};
    std::unordered_map<std::string, int> frame_id_map{{"base", 1}};
    std::unordered_map<std::string, std::string> recipe_wdat_map{
        {"WELD_PROGRAM_01", "WDAT_DEMO_01"}};
};

//语义映射器，负责将Goal中的原始数据打包为IR结构体
class SemanticMapper {
public:
    SemanticMapper() {
        read_config();
    }
    ~SemanticMapper() = default;

    //将 Goal 中的 AtomicInstruction[] 解析并映射为 KukaArcTechJobIR
    ErrorCode map(
        const atomic_adapter_interfaces::action::ExecuteBackendJob::Goal &goal,
        KukaArcTechJobIR &out_ir,
        std::string &error_message) const;

private:
    void read_config();

    std::string make_program_name(const std::string &job_id) const;

    ErrorCode bind_tool_and_base(
        const std::string &tool_name,
        const std::string &frame_name,
        KukaArcTechJobIR &ir,
        std::string &error_message) const;

    MapperConfig config_;
};

#endif // SEMANTIC_MAPPER_H
