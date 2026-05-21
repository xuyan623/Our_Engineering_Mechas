#ifndef NEW_ROBOT_INPUT_TASK_JUDGE_STUB_H
#define NEW_ROBOT_INPUT_TASK_JUDGE_STUB_H

#include "task/input_task/input_task.h"

/* 这里只保留 owner 位，不在本轮真的打开或消费 USART3。
 * 下一轮真正接裁判/图传输入时，直接在 input_task 里补这一源即可。
 */
#define INPUT_TASK_JUDGE_USART3_BAUDRATE (115200u)

void input_task_judge_stub_reset_runtime(InputTaskJudgeStubDebugState* runtime);

#endif
