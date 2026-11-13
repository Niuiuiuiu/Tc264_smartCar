#include "motor_ctrl.h"
#include "drv_motor.h"
#include "lib_pid.h"
#include "car_cfg.h"
#include <math.h>
#include <stdlib.h>

motor_ctrl_t motor_left;
motor_ctrl_t motor_right;


void motor_ctrl_init(void){
    drv_motor_init(&motor_left.info,MOTOR_LEFT_PWM,MOTOR_LEFT_DIR,LEFT,MOTOR_LEFT_ENCODER_TIM,
        MOTOR_LEFT_ENCODER_CH1,MOTOR_LEFT_ENCODER_CH2);
    lib_pid_init(&motor_left.pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, (float)PWM_DUTY_MAX, 0.0f);
    drv_motor_init(&motor_right.info,MOTOR_RIGHT_PWM,MOTOR_RIGHT_DIR,RIGHT,MOTOR_RIGHT_ENCODER_TIM,
        MOTOR_RIGHT_ENCODER_CH1,MOTOR_RIGHT_ENCODER_CH2);
    lib_pid_init(&motor_right.pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, (float)PWM_DUTY_MAX, 0.0f);
}

void motor_ctrl(motor_ctrl_t* motor, float target_speed, float dt){
    float speed;
    float pwm;

    // Reject invalid parameters before reading or clearing the encoder.
    if (motor == NULL || !isfinite(dt) || dt <= 0.0f) {
        return;
    }

    drv_motor_get_encoder(&motor->info);
    speed = (float)motor->info.encode ;
    if (speed > (float)INT16_MAX) {
        speed = (float)INT16_MAX;
    } else if (speed < (float)INT16_MIN) {
        speed = (float)INT16_MIN;
    }
    motor->info.speed = (int16_t)speed;

    pwm = lib_pid_ff_calc(&motor->pid, (float)target_speed, (float)motor->info.speed,0.0f,0.0f, dt);
    if (!isfinite(pwm)) {
        lib_pid_reset(&motor->pid);
        drv_motor_set_duty(&motor->info, 0U);
        return;
    }
    if (pwm > (float)PWM_DUTY_MAX) {
        pwm = (float)PWM_DUTY_MAX;
    } else if (pwm < -(float)PWM_DUTY_MAX) {
        pwm = -(float)PWM_DUTY_MAX;
    }
    motor->info.pwm_output = (int16_t)pwm;
    if(motor->info.pwm_output >= 0){ drv_motor_set_dir(&motor->info, LEFT);}
    else{drv_motor_set_dir(&motor->info, RIGHT);}
    drv_motor_set_duty(&motor->info, abs(motor->info.pwm_output));
}
