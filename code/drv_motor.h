#ifndef _MOTOR_H__
#define _MOTOR_H__

#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"
#include "zf_driver_encoder.h"
#include <stdint.h>

typedef enum{
    LEFT = 0,
    RIGHT = 1
}MOTOR_DIR;

typedef struct{
    pwm_channel_enum pwm_pin;
    gpio_pin_enum  dir_pin;
    encoder_index_enum encoder_timer;
    encoder_channel1_enum encoder_channel1;
    encoder_channel2_enum encoder_channel2;
}motor_pin_info;

typedef struct{
    motor_pin_info pin_info;
    MOTOR_DIR dir;
    int16_t encode; //这个理论上被当作速度用了
    int16_t speed;
    int16_t pwm_output;
}motor_info;

/**
 * @brief 初始化电机
 * @param motor 电机实例指针
 * @param pwm_pin 控制电机的pwm引脚
 * @param dir_pin 控制电机方向的引脚
 * @param dir 电机方向
 * @param encoder_timer 编码器时钟
 * @param encoder_channel1 编码器通道1
 * @param encoder_channel2 编码器通道2
 */
void drv_motor_init(motor_info* motor,pwm_channel_enum pwm_pin,gpio_pin_enum dir_pin,MOTOR_DIR dir,
encoder_index_enum encoder_timer,encoder_channel1_enum encoder_channel1,encoder_channel2_enum encoder_channel2);

/**
 * @brief 设置电机占空比
 * @param motor 电机实例指针
 * @param duty 要设置的占空比 真实占空比是 duty/最大占空比
 */
void drv_motor_set_duty(motor_info* motor,uint32_t duty);

/**
 * @brief 设置电机方向
 * @param motor 电机实例指针
 * @param dir 要设置的方向
 */
void drv_motor_set_dir(motor_info* motor,MOTOR_DIR dir);

/**
 * @brief 获取电机编码器值
 * @param motor 电机实例指针
 */
void drv_motor_get_encoder(motor_info* motor);

#endif /* _MOTOR_H__ */