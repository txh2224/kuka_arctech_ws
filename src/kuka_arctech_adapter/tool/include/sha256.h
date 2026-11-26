#ifndef TOOL_SHA256_H
#define TOOL_SHA256_H

#include <string>

namespace tool {

// 按输入的原始字节计算 SHA-256，返回 64 个小写十六进制字符。
// 本项目用它计算生成后的 SRC 和 DAT 文件摘要，并将摘要写入 artifact_manifest.json。
// 导出或传输后可以重新计算摘要，与清单比对，核对文件内容是否一致。
std::string sha256_hex(const std::string &data);

}  // namespace tool

#endif  // TOOL_SHA256_H
