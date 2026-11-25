#ifndef CAR_CFG_H
#define CAR_CFG_H

#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"

/** @brief 整车配置宏；几何尺寸统一使用 m。 */

/* 车架几何参数：来自 MiniCar_140mm 模型说明，实车装配后需复核。 */
#define CAR_OVERALL_WIDTH_M          0.14000f  // 含轮胎的整车外宽。
#define CAR_WHEEL_DIAMETER_M         0.02430f  // 轮胎名义外径；受载滚动直径待实测。
#define CAR_WHEEL_RADIUS_M           (CAR_WHEEL_DIAMETER_M * 0.5f)
#define CAR_WHEELBASE_M              0.02550f  // 同侧前后车轮中心距。
#define CAR_WHEEL_WIDTH_M            0.01550f  // 轮胎宽度，模型推定值。
#define CAR_TRACK_WIDTH_M            (CAR_OVERALL_WIDTH_M - CAR_WHEEL_WIDTH_M) // 左右轮中心距，推算为 0.12450 m。

/* 四轮滑移转向的轮距修正：1.0 为初始值，需根据实车转角标定。 */
#define CAR_TRACK_WIDTH_SCALE        1.0f
#define CAR_EFFECTIVE_TRACK_WIDTH_M  (CAR_TRACK_WIDTH_M * CAR_TRACK_WIDTH_SCALE)  //有效轮距

/* 外部齿轮传动：小齿轮轴转速 / 车轮转速 = 41 / 10 = 4.1。
 * 不包含电机内部减速机构；编码器在轮轴侧时不再应用此减速比。
 */
//#define CAR_MOTOR_PINION_TEETH        10U
//#define CAR_WHEEL_GEAR_TEETH          41U
//#define CAR_EXTERNAL_GEAR_RATIO      ((float)CAR_WHEEL_GEAR_TEETH / (float)CAR_MOTOR_PINION_TEETH)

#define CAR_ENCODER_COUNTS_PER_WHEEL_REV  1200.0f  // 编码器每转脉冲数, 目前是随便乱给的

#define CAR_ENCODER_MAX_COUNTS_PER_DT  30000.0f  // 编码器每dt的最大脉冲数

/* 左右电机 PWM、方向和编码器引脚。 */

#define MOTOR_LEFT_PWM   ATOM3_CH5_P11_10
#define MOTOR_LEFT_DIR   P11_9
#define MOTOR_LEFT_ENCODER_TIM TIM4_ENCODER
#define MOTOR_LEFT_ENCODER_CH1 TIM4_ENCODER_CH1_P02_8
#define MOTOR_LEFT_ENCODER_CH2 TIM4_ENCODER_CH2_P00_9

#define MOTOR_RIGHT_PWM  ATOM3_CH7_P11_12
#define MOTOR_RIGHT_DIR  P11_11
#define MOTOR_RIGHT_ENCODER_TIM TIM6_ENCODER
#define MOTOR_RIGHT_ENCODER_CH1 TIM6_ENCODER_CH1_P20_3
#define MOTOR_RIGHT_ENCODER_CH2 TIM6_ENCODER_CH2_P20_0

#endif // CAR_CFG_H