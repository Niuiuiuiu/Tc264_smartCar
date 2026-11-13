#include "chassis.h"
#include "car_cfg.h"
#include "motor_ctrl.h"
#include <math.h>

#define M_PI 3.14159265358979323846f

extern motor_ctrl_t motor_left , motor_right;



//可能后面会加显示器或者按键之类
void chassis_init(void)
{
    motor_ctrl_init();
}

void chassis_ctrl(float target_vel,float target_omega, float dt){

    float target_speed_left = ((target_vel - target_omega * 
        CAR_EFFECTIVE_TRACK_WIDTH_M * 0.5f) );
    float target_speed_right = ((target_vel + target_omega *
         CAR_EFFECTIVE_TRACK_WIDTH_M * 0.5f) );

    float target_count_left = target_speed_left * dt * CAR_ENCODER_COUNTS_PER_WHEEL_REV
        / (2.0f * M_PI * CAR_WHEEL_RADIUS_M);
    float target_count_right = target_speed_right * dt * CAR_ENCODER_COUNTS_PER_WHEEL_REV
        / (2.0f * M_PI * CAR_WHEEL_RADIUS_M);

    /* 两侧计数绝对值的最大值。 */
    float max_count = fmaxf(fabsf(target_count_left),
                        fabsf(target_count_right));

    /* 超限时，两侧等比例缩小，保留正负方向。 */
    if (max_count > CAR_ENCODER_MAX_COUNTS_PER_DT)
    {
        float scale = CAR_ENCODER_MAX_COUNTS_PER_DT / max_count;

        target_count_left  *= scale;
        target_count_right *= scale;
    }

    motor_ctrl(&motor_left, target_count_left, dt);
    motor_ctrl(&motor_right, target_count_right, dt);
}
