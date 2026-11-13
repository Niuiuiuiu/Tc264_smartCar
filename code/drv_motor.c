#include "drv_motor.h"


#define MOTOR_LEFT_PWM   ATOM2_CH4_P11_9
#define MOTOR_LEFT_DIR   ATOM2_CH3_P11_10

#define MOTOR_RIGHT_PWM  ATOM2_CH5_P11_11
#define MOTOR_RIGHT_DIR  ATOM2_CH6_P11_12

#define MOTOR_PWM_OUTPUT_FREQ  1700  //电机输出pwm频率
#define MOTOR_PWM_MAX_DUTY   PWM_DUTY_MAX   //电机输出pwm占空比最大值
#define MOTOR_ENCODER_MAX  1200  //转一圈电机编码器最大值（暂时没有用）

    
void drv_motor_init(motor_info* motor,pwm_channel_enum pwm_pin,gpio_pin_enum dir_pin,MOTOR_DIR dir,
encoder_index_enum encoder_timer,encoder_channel1_enum encoder_channel1,encoder_channel2_enum encoder_channel2){
    memset(motor,0,sizeof(motor_info));
    motor->pin_info.pwm_pin = pwm_pin;
    motor->pin_info.dir_pin = dir_pin;
    motor->pin_info.encoder_timer = encoder_timer;
    motor->pin_info.encoder_channel1 = encoder_channel1;
    motor->pin_info.encoder_channel2 = encoder_channel2;
    motor->dir = dir;
    encoder_dir_init(encoder_timer,encoder_channel1,encoder_channel2);
    encoder_clear_count(encoder_timer);
    pwm_init(pwm_pin,MOTOR_PWM_OUTPUT_FREQ,0);
    gpio_init(dir_pin,GPO,GPIO_LOW, GPO_PUSH_PULL);
    gpio_set_level(dir_pin,(uint8_t)motor->dir);
}

void drv_motor_set_duty(motor_info* motor,uint32_t duty){
    if(duty > MOTOR_PWM_MAX_DUTY) duty = MOTOR_PWM_MAX_DUTY;
    motor->pwm_output = (int16_t)duty;
    pwm_set_duty(motor->pin_info.pwm_pin,duty);
}

void drv_motor_set_dir(motor_info* motor,MOTOR_DIR dir){
    motor->dir = dir;
    gpio_set_level(motor->pin_info.dir_pin,(uint8_t)motor->dir);
}

void drv_motor_get_encoder(motor_info* motor){  
    motor->encode = encoder_get_count(motor->pin_info.encoder_timer);
    encoder_clear_count(motor->pin_info.encoder_timer);
}

void drv_motor_get_encoder_e(motor_info* motor){  
    motor->encode = encoder_get_count(motor->pin_info.encoder_timer);
}

