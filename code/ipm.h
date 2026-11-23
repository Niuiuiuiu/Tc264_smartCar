/* ============================================================================
 * ipm.h  ——  逆透视变换（Inverse Perspective Mapping）
 *            像素坐标 (col,row)  ->  地面米制坐标 (x,y)
 *
 * 为什么要做这件事：
 *   摄像头看到的是像素，透视会让"1 个像素 = 多少厘米"随位置剧烈变化。
 *   但控制（横向偏差、曲率、航向角）必须在米制坐标里算。
 *   H 就是那个翻译器；标定就是把这 9 个数从实验里求出来。
 *
 * 符号约定（全工程只在这里定义一次，改就改这里）：
 *   原点 = 车中心在地面的投影
 *   x > 0 ：车头正前方     单位：米
 *   y > 0 ：车身左侧       单位：米
 * ==========================================================================*/

/* ============================================================================
 * 开发须知
 *
 *   这一节不涉及推导，只说"怎么用"和"别踩什么坑"。
 *   为什么成立，另见《TC264摄像头坐标标定 原理说明》文档。
 *
 * 1) 输入输出
 *      line_point_t pt;                    // line_detect() 的输出
 *      ipm_ground_point_t g;
 *      if (ipm_point_to_ground(&pt, &g) == IPM_OK) { ... }
 *
 *      g.x   车头方向为正，单位米
 *      g.y   车身左侧为正，单位米
 *      原点  车中心在地面的投影
 *
 * 2) 返回值不是 IPM_OK，就丢掉这个点，绝对不要塞进控制环
 *      IPM_ERR_HORIZON   该像素在地平线上，几何上无解
 *      IPM_ERR_RANGE     算得出来，但超出你定义的信任范围
 *
 * 3) 上电必须调一次 ipm_init()，否则反向变换返回 IPM_ERR_NO_INIT
 *
 * 4) IPM_H 目前是占位数据，只能跑通链路，绝不能跑车。
 *      标定后把 tools/calib_ipm.py 打印的数组整块替换掉。
 *
 * 5) IPM_H 绑定的是"摄像头姿态 + 地面"。支架松了、碰了、拆了，
 *      一律重新标定。典型症状：车跑到某一段莫名其妙画龙。
 *
 * 调参旋钮（改这 4 个宏 = 改"你信任哪块视野"）
 *      ipm.c 里的 IPM_X_MIN / IPM_X_MAX / IPM_Y_MIN / IPM_Y_MAX
 *
 * 自检
 *      上电调 ipm_selftest()，重点看两个数：
 *        OUT_OF_RANGE   不为 0 是正常的（地平线附近会被拒）
 *        usable rows    必须覆盖你真正要打舵的那片区域
 *
 * 性能与用法
 *      一次变换 = 9 次乘法 + 2 次除法，放心用 float，不用优化。
 *      只转要参与控制的点，不要把 188x120 全图都转一遍。
 *
 *      注意：line_detect() 的输出是"近 -> 远"排列，
 *            points[0] 是最近的行，只读 points[0 .. count-1]。
 * ==========================================================================*/

#ifndef IPM_H_INCLUDED
#define IPM_H_INCLUDED

#include <stdint.h>
#include "line_detect.h"        /* line_point_t 与 MT9V03X_1_W/H */

typedef struct
{
    float x;                    /* 车前方，米 */
    float y;                    /* 偏左为正，米 */
} ipm_ground_point_t;

typedef enum
{
    IPM_OK           = 0,
    IPM_ERR_ARG      = 1,       /* 传了空指针 */
    IPM_ERR_HORIZON  = 2,       /* 分母≈0：该像素在地平线上，几何上无解 */
    IPM_ERR_RANGE    = 3,       /* 算得出来，但落在标定有效范围之外 */
    IPM_ERR_NO_INIT  = 4,       /* 忘了先调 ipm_init() */
    IPM_ERR_SINGULAR = 5,       /* H 退化，求不了逆矩阵 */
} ipm_status_t;

/* 上电调一次：预先算好 H 的逆矩阵。不调的话反向变换不可用 */
void ipm_init(void);

/* 核心变换：像素 -> 地面 */
ipm_status_t ipm_uv_to_ground(float col, float row, ipm_ground_point_t *out);

/* 直接吃 line_detect() 的输出 */
ipm_status_t ipm_point_to_ground(const line_point_t *in, ipm_ground_point_t *out);

/* 反向：地面 -> 像素（画鸟瞰图、串口回显用） */
ipm_status_t ipm_ground_to_uv(float x, float y, float *col, float *row);

/* 标定体检：把一个标定点代回 H，返回误差米数；负数表示该点算不出来 */
float ipm_residual_m(float col, float row, float x_ref, float y_ref);

/* 自检：打印四个角映射 + 全图有效点数统计。需要工程支持 printf */
void ipm_selftest(void);

#endif /* IPM_H_INCLUDED */
