#include "foc_handle.h"
#include "util.h"
#include "usr_config.h"
#include "systick.h"
#include "tim.h"
#include "drv8323.h"

FOCStruct Foc;

void FOC_zero_current(FOCStruct *foc)
{
	
	Foc.zero_flag = 0;
	
	int adc_sum_a = 0;
	int adc_sum_b = 0;
	int n = 1000;
	for (int i = 0; i < n; i++)
	{ // Average n samples of the ADC
		SYSTICK_delay_us(100);
		adc_sum_a += foc->adc_phase_a;
		adc_sum_b += foc->adc_phase_b;
	}
	foc->adc_phase_a_offset = adc_sum_a / n;
	foc->adc_phase_b_offset = adc_sum_b / n;
//	
//	foc->adc_phase_a_offset = 2047;
//	foc->adc_phase_b_offset = 2001;
//	foc->adc_phase_c_offset = 2097;
	
	Foc.zero_flag = 1;
}

void PWMC_switch_on_pwm(void)
{
	/* Set all duty to 50% */
	TIM1->CCR3 = (PWM_ARR>>1);
	TIM1->CCR2 = (PWM_ARR>>1);
	TIM1->CCR1 = (PWM_ARR>>1);
	
	/* wait for a new PWM period */
	LL_TIM_ClearFlag_UPDATE(TIM1);
	while ( LL_TIM_IsActiveFlag_UPDATE(TIM1) == 0u )
	{}
	LL_TIM_ClearFlag_UPDATE(TIM1);
    
    // 启用主输出（MOE）
//    LL_TIM_EnableAllOutputs(TIM1);
//    
//    // 使能通道1主输出和互补输出
//    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
//    
//    // 使能通道2主输出和互补输出
//    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
//    
//    // 使能通道3主输出和互补输出
//    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
//    
//    // 使能定时器计数器
//    LL_TIM_EnableCounter(TIM1);
	
		DRV8323_enable_gd();
}

void PWMC_switch_off_pwm(void)
{
//	  LL_TIM_OC_SetCompareCH1(TIM1, PWM_ARR >> 1);
//    LL_TIM_OC_SetCompareCH2(TIM1, PWM_ARR >> 1);
//    LL_TIM_OC_SetCompareCH3(TIM1, PWM_ARR >> 1);
	
		DRV8323_disable_gd();
	
	/* Set all duty to 50% */
	TIM1->CCR3 = (PWM_ARR>>1);
	TIM1->CCR2 = (PWM_ARR>>1);
	TIM1->CCR1 = (PWM_ARR>>1);
	
	/* wait for a new PWM period */
	LL_TIM_ClearFlag_UPDATE(TIM1);
	while ( LL_TIM_IsActiveFlag_UPDATE(TIM1) == 0u )
	{}
	LL_TIM_ClearFlag_UPDATE(TIM1);;
}

//void PWMC_switch_on_pwm(void)
//{
//	/* Set all duty to 50% */
//	TIM1->CCR3 = (PWM_ARR>>1);
//	TIM1->CCR2 = (PWM_ARR>>1);
//	TIM1->CCR1 = (PWM_ARR>>1);
//	
//	/* wait for a new PWM period */
////	LL_TIM_ClearFlag_UPDATE(TIM1);
////	while ( LL_TIM_IsActiveFlag_UPDATE(TIM1) == 0u )
////	{}
////	LL_TIM_ClearFlag_UPDATE(TIM1);

//	/* Main PWM Output Enable */
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
//}

//void PWMC_switch_off_pwm(void)
//{
//	/* Main PWM Output Disable */
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
//    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
//	
//	/* wait for a new PWM period */
////	LL_TIM_ClearFlag_UPDATE(TIM1);
////	while ( LL_TIM_IsActiveFlag_UPDATE(TIM1) == 0u )
////	{}
////	LL_TIM_ClearFlag_UPDATE(TIM1);
//}

void FOC_arm(void)
{
	uint32_t prim = cpu_enter_critical();

	PWMC_switch_on_pwm();

	cpu_exit_critical(prim);
}

void FOC_disarm(void)
{
	uint32_t prim = cpu_enter_critical();

	PWMC_switch_off_pwm();

	cpu_exit_critical(prim);
}

void FOC_reset(FOCStruct *foc)
{
	foc->i_d_filt = 0;
	foc->i_q_filt = 0;
	foc->current_ctrl_integral_d = 0;
	foc->current_ctrl_integral_q = 0;
}

void apply_voltage_timings(float vbus, float v_d, float v_q, float pwm_phase)
{
	// Modulation
	float V_to_mod = 1.0f / ((2.0f / 3.0f) * vbus);
	float mod_d = V_to_mod * v_d;
	float mod_q = V_to_mod * v_q;

	// Vector modulation saturation, lock integrator if saturated
	float mod_scalefactor = 0.80f * SQRT3_BY_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
	if (mod_scalefactor < 1.0f)
	{
		mod_d *= mod_scalefactor;
		mod_q *= mod_scalefactor;
	}

	// Inverse park transform
	float mod_alpha;
	float mod_beta;
	inverse_park(mod_d, mod_q, pwm_phase, &mod_alpha, &mod_beta);

	// SVM
	float dtc_a, dtc_b, dtc_c;
	svm(mod_alpha, mod_beta, &dtc_a, &dtc_b, &dtc_c);

	// Apply duty
	TIM1->CCR3 = (uint16_t)(dtc_a * (float)PWM_ARR);
	TIM1->CCR2 = (uint16_t)(dtc_b * (float)PWM_ARR);
	TIM1->CCR1 = (uint16_t)(dtc_c * (float)PWM_ARR);
	// Apply duty
//	htim1.Instance->CCR3 = (uint16_t)(dtc_a * (float)PWM_ARR);
//	htim1.Instance->CCR2 = (uint16_t)(dtc_b * (float)PWM_ARR);
//	htim1.Instance->CCR1 = (uint16_t)(dtc_c * (float)PWM_ARR);

}

void FOC_current(FOCStruct *foc, float Id_des, float Iq_des, float I_phase, float pwm_phase)
{

	// Clarke transform
	float i_alpha, i_beta;
	clarke_transform(foc->i_a, foc->i_b, foc->i_c, &i_alpha, &i_beta);

	// Park transform
	float i_d, i_q;
	park_transform(i_alpha, i_beta, I_phase, &i_d, &i_q);

	// Current Filter used for report
	foc->i_d_filt = 0.95f * foc->i_d_filt + 0.05f * i_d;
	foc->i_q_filt = 0.95f * foc->i_q_filt + 0.05f * i_q;

	// Apply PI control
	float Ierr_d = Id_des - i_d;
	float Ierr_q = Iq_des - i_q;
	float v_d = UsrConfig.current_ctrl_p_gain * Ierr_d + foc->current_ctrl_integral_d;
	float v_q = UsrConfig.current_ctrl_p_gain * Ierr_q + foc->current_ctrl_integral_q;

	// Modulation
	float mod_to_V = (2.0f / 3.0f) * foc->v_bus;
	float V_to_mod = 1.0f / mod_to_V;
	float mod_d = V_to_mod * v_d;
	float mod_q = V_to_mod * v_q;

	// Vector modulation saturation, lock integrator if saturated
	float mod_scalefactor = 0.8f * SQRT3_BY_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
	if (mod_scalefactor < 1.0f)
	{
		mod_d *= mod_scalefactor;
		mod_q *= mod_scalefactor;
		foc->current_ctrl_integral_d *= 0.99f;
		foc->current_ctrl_integral_q *= 0.99f;
	}
	else
	{
		foc->current_ctrl_integral_d += Ierr_d * (UsrConfig.current_ctrl_i_gain * DT);
		foc->current_ctrl_integral_q += Ierr_q * (UsrConfig.current_ctrl_i_gain * DT);
	}

	// Compute estimated bus current
	foc->i_bus_filt = mod_d * foc->i_d_filt + mod_q * foc->i_q_filt;

	// Inverse park transform
	float mod_alpha;
	float mod_beta;
	inverse_park(mod_d, mod_q, pwm_phase, &mod_alpha, &mod_beta);

	// SVM
	float dtc_a, dtc_b, dtc_c;
	svm(mod_alpha, mod_beta, &dtc_a, &dtc_b, &dtc_c);

	// Apply duty
	TIM1->CCR3 = (uint16_t)(dtc_a * (float)PWM_ARR);
	TIM1->CCR2 = (uint16_t)(dtc_b * (float)PWM_ARR);
	TIM1->CCR1 = (uint16_t)(dtc_c * (float)PWM_ARR);
}
