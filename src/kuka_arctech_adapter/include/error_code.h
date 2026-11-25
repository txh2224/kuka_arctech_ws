#ifndef ERROR_CODE_H
#define ERROR_CODE_H

#include <stdexcept>

// 内部成功状态与规格书第 12 节定义的业务错误码
enum class ErrorCode {
    OK, // 内部成功状态，写入 Action Result 时对应空字符串
    KA_JOB_INVALID, // 作业缺参、指令顺序非法或无法映射到 KUKA 后端
    KA_TEMPLATE_NOT_FOUND, // 找不到匹配目标版本或工艺的黄金模板
    KA_TEMPLATE_RENDER_FAILED, // 黄金模板实例化失败，如占位符或格式错误
    KA_STATIC_CHECK_FAILED, // 生成的程序包未通过静态校验，禁止部署
    KA_KRC_UNREACHABLE, // 无法连接或探测 KRC
    KA_DEPLOY_FAILED, // 程序包部署失败，保留原始错误证据
    KA_PROGRAM_START_FAILED, // 控制器选择或启动程序失败，保留原始错误证据
    KA_SEAM_PERMISSION_TIMEOUT, // 等待 PLC 或上级系统的焊缝许可握手超时
    KA_ARCTECH_FAULT, // ArcTech 焊接故障，在 evidence 中保留原始故障码
    KA_PROGRAM_TIMEOUT, // 程序运行超时，应请求停止
    KA_STOP_FAILED, // 停止未获确认，进入 UNKNOWN 或 FAILED 并提示人工检查
};

//写入 Action Result 前转换成规格书规定的字符串
constexpr const char *to_error_code_string(ErrorCode code) {
    switch (code) {
        case ErrorCode::OK:
            return "";
        case ErrorCode::KA_JOB_INVALID:
            return "KA_JOB_INVALID";
        case ErrorCode::KA_TEMPLATE_NOT_FOUND:
            return "KA_TEMPLATE_NOT_FOUND";
        case ErrorCode::KA_TEMPLATE_RENDER_FAILED:
            return "KA_TEMPLATE_RENDER_FAILED";
        case ErrorCode::KA_STATIC_CHECK_FAILED:
            return "KA_STATIC_CHECK_FAILED";
        case ErrorCode::KA_KRC_UNREACHABLE:
            return "KA_KRC_UNREACHABLE";
        case ErrorCode::KA_DEPLOY_FAILED:
            return "KA_DEPLOY_FAILED";
        case ErrorCode::KA_PROGRAM_START_FAILED:
            return "KA_PROGRAM_START_FAILED";
        case ErrorCode::KA_SEAM_PERMISSION_TIMEOUT:
            return "KA_SEAM_PERMISSION_TIMEOUT";
        case ErrorCode::KA_ARCTECH_FAULT:
            return "KA_ARCTECH_FAULT";
        case ErrorCode::KA_PROGRAM_TIMEOUT:
            return "KA_PROGRAM_TIMEOUT";
        case ErrorCode::KA_STOP_FAILED:
            return "KA_STOP_FAILED";
    }
    throw std::invalid_argument("未知的 Adapter 错误码。");
}

#endif  // ERROR_CODE_H
