#ifndef _CHASSIS_H_
#define _CHASSIS_H_


/**
 * @brief 底盘初始化
 * 
 */
void chassis_init(void);

/**
 * @brief 执行一次底盘控制，将整车目标速度解算到左右轮目标速度，并调用电机控制函数
 * @param target_vel 目标速度，单位m/s
 * @param target_omega 目标角速度，单位rad/s
 * @param dt 每隔dt的时间间隔，单位s
 */
void chassis_ctrl(float target_vel,float target_omega, float dt);


#endif // _CHASSIS_H_