#ifndef JOB_VALIDATOR_H
#define JOB_VALIDATOR_H

//我其实不是很理解为什么任务书要求把这个检查任务是否合法的函数单独分一个文件
//可能以后检查任务是否合法的逻辑会变得很复杂吧
#include <string>

#include <atomic_adapter_interfaces/action/execute_backend_job.hpp>

class JobValidator {
public:
    //检查转成 IR 前所需的 Goal 基本字段。
    bool validate(
        const atomic_adapter_interfaces::action::ExecuteBackendJob::Goal &goal,
        std::string &error_message) const;
};

#endif // JOB_VALIDATOR_H
