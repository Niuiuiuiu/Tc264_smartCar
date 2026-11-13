#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "drv_motor.h"
#include "lib_pid.h"

typedef struct{
    motor_info info;
    lib_pid_t pid;
} motor_ctrl_t;

/**
 * @brief 初始化两个电机实例
 */
void motor_ctrl_init(void);

/**
 * @brief 控制电机转速 （目前速度目标是每隔dt的脉冲数）
 * @param motor 电机实例指针
 * @param target_speed 目标速度
 * @param dt 时间间隔 单位是秒
 * @note 暂时对外暴露用作调试使用
 */
void motor_ctrl(motor_ctrl_t *motor, float target_speed, float dt);






#endif /* _MOTOR_CTRL_H_ */
