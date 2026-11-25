#ifndef TEMPLATE_ENGINE_H
#define TEMPLATE_ENGINE_H

#include <string>

#include "error_code.h"
#include "job_ir.h"
#include "krc_transport.h"

// 规格书第 6、8 节：黄金模板选择与白名单占位符实例化引擎
class TemplateEngine {
public:
    TemplateEngine();
    explicit TemplateEngine(const std::string &templates_root);

    // 根据 IR 选择模板并用白名单占位符渲染出 .src / .dat 程序包
    ErrorCode render(
        const KukaArcTechJobIR &job_ir,
        ProgramPackage &out_pkg,
        std::string &error_message) const;

private:
    std::string templates_root_;
};

#endif  // TEMPLATE_ENGINE_H
