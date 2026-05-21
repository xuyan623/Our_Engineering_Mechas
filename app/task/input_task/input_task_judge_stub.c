#include "task/input_task/input_task_judge_stub.h"

#include <string.h>

void input_task_judge_stub_reset_runtime(InputTaskJudgeStubDebugState* runtime)
{
    if (runtime == OM_NULL)
    {
        return;
    }

    memset((void*)runtime, 0, sizeof(*runtime));
}
