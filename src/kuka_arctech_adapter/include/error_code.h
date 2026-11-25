#ifndef ERROR_CODE_H
#define ERROR_CODE_H

#include <stdexcept>

//规格书第 12 节定义的业务错误码
enum class ErrorCode {
    OK,
    KA_JOB_INVALID,
    KA_TEMPLATE_NOT_FOUND,
    KA_TEMPLATE_RENDER_FAILED,
    KA_STATIC_CHECK_FAILED,
    KA_KRC_UNREACHABLE,
    KA_DEPLOY_FAILED,
    KA_PROGRAM_START_FAILED,
    KA_SEAM_PERMISSION_TIMEOUT,
    KA_ARCTECH_FAULT,
    KA_PROGRAM_TIMEOUT,
    KA_STOP_FAILED,
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
