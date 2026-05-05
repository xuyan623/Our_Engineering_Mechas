#ifndef NEW_ROBOT_DJI_MOTOR_H
#define NEW_ROBOT_DJI_MOTOR_H

/* B0 阶段不重复实现 DJI 驱动，
 * 只把框架现成接口固定暴露到项目目标路径下。
 * 后续统一抽象、控制计算与总线发送收敛，都放到 driver/motor 层处理。
 */
#include "drivers/motor/vendors/dji/dji_motor_drv.h"

#endif
