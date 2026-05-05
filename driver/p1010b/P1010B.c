#include "driver/p1010b/P1010B.h"

/* 当前 B2 只做路径适配，不增加二次封装实现。
 * 这样后续模块统一通过 new_robot_code/driver/p1010b 引用 P1010B 驱动，
 * 同时底层功能仍完全复用框架已有实现。
 */
