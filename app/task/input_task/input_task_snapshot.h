#ifndef NEW_ROBOT_INPUT_TASK_SNAPSHOT_H
#define NEW_ROBOT_INPUT_TASK_SNAPSHOT_H

#include <stdint.h>

/* 自定义控制器对外快照固定导出 6 轴角度。 */
#define INPUT_TASK_CUSTOM_CONTROLLER_ANGLE_COUNT (6u)

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t l;
    uint8_t r;
} InputMouseSnapshot;

/* input_task 对外发布的正式 RC 快照 DTO。 */
typedef struct
{
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    uint8_t sw1;
    uint8_t sw2;
    uint16_t iw;
    uint8_t online;
    InputMouseSnapshot mouse;
    uint16_t keyboard_bits;
} InputRcSnapshot;

/* input_task 对外发布的正式自定义控制器快照 DTO。 */
typedef struct
{
    uint8_t online;
    uint8_t work_mode;
    float angle_deg[INPUT_TASK_CUSTOM_CONTROLLER_ANGLE_COUNT];
} InputCustomControllerSnapshot;

#endif
