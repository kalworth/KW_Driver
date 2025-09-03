#include "pll.h"
#include "util.h"

PLLSpeedEstimatorRad Pll;


// 初始化
void pll_speed_estimator_rad_init(PLLSpeedEstimatorRad* pll, float bandwidth) {
    pll->pll_kp = 2.0f * bandwidth;
    pll->pll_ki = 0.25f * pll->pll_kp * pll->pll_kp;
    pll->pos_estimate_rad = 0.0f;
    pll->vel_estimate_rad = 0.0f;
    pll->pos_estimate = 0.0f;
    pll->vel_estimate = 0.0f;
}

// 单步更新
void pll_speed_estimator_rad_update(
    PLLSpeedEstimatorRad* pll,
    float abs_angle_rad,   // 当前绝对角度，单位：弧度，范围0~2π
    float dt               // 控制周期，单位：秒
) {
    // 1. 预测当前位置
    pll->pos_estimate_rad += dt * pll->vel_estimate_rad;

    // 2. 计算位置误差（弧度，考虑环绕）
    float delta_pos = wrap_pm_pi(abs_angle_rad - pll->pos_estimate_rad);

    // 3. PLL反馈修正
    pll->pos_estimate_rad += dt * pll->pll_kp * delta_pos;
    pll->vel_estimate_rad += dt * pll->pll_ki * delta_pos;

    // 5. 输出
    pll->pos_estimate_raw = pll->pos_estimate_rad;
    pll->vel_estimate_raw = pll->vel_estimate_rad;
    // 保证输出在0~2π
    pll->pos_estimate = wrap_pm_pi(pll->pos_estimate_rad) + M_PI;
	
//	//    // 4. 速度抖动抑制
//    if (fabsf(pll->vel_estimate_rad) < 0.5f * dt * pll->pll_ki) {
//        pll->vel_estimate = 0.0f;
//    }
//		else{
//			pll->vel_estimate = pll->vel_estimate_rad;}
		pll->vel_estimate = pll->vel_estimate_rad;
}
