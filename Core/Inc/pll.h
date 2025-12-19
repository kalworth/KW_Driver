#ifndef __PLL_H__
#define __PLL_H__

#include "main.h"
typedef struct {
    float pos_estimate;      // 位置估算（弧度，0~2π）
    float vel_estimate;      // 速度估算（弧度/秒）
    float pos_estimate_raw;  // 原始位置估算（弧度）
    float vel_estimate_raw;  // 原始速度估算（弧度/秒）
    float pll_kp;            // PLL比例增益
    float pll_ki;            // PLL积分增益
    float pos_estimate_rad;
    float vel_estimate_rad;
} PLLSpeedEstimatorRad;

//extern PLLSpeedEstimatorRad Pll;

void pll_speed_estimator_rad_init(PLLSpeedEstimatorRad* pll, float bandwidth);
void pll_speed_estimator_rad_update(
    PLLSpeedEstimatorRad* pll,
    float abs_angle_rad,   // 当前绝对角度，单位：弧度，范围0~2π
    float dt               // 控制周期，单位：秒
);
#endif
