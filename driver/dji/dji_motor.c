#include "driver/dji/dji_motor.h"

/* 当前 B0 只做路径适配，不增加二次封装实现。
 * 这样后续模块统一通过 new_robot_code/driver/dji 引用 DJI 驱动，
 * 同时底层功能仍完全复用框架已有实现。
 */
