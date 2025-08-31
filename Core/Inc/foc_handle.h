#ifndef __FOC_HANDLE_H
#define __FOC_HANDLE_H

#include "main.h"

typedef struct{
	uint16_t adc_phase_a, adc_phase_b,adc_vbus; // Raw ADC Values
	float hall_adc[6];
	float v_bus;								 // DC link voltage
	float i_a, i_b, i_c;						 // Phase currents
	float i_d_filt, i_q_filt, i_bus_filt, i_q_tar;		 // D/Q currents

	float current_ctrl_integral_d, current_ctrl_integral_q; // Current error integrals
	int adc_phase_a_offset, adc_phase_b_offset;				// ADC offsets
	float zero_flag;
} FOCStruct;

extern FOCStruct Foc;

void FOC_zero_current(FOCStruct *foc);

void FOC_arm(void);

void FOC_disarm(void);

void FOC_reset(FOCStruct *foc);

void apply_voltage_timings(float vbus, float v_d, float v_q, float pwm_phase);

void FOC_current(FOCStruct *foc, float Id_des, float Iq_des, float I_phase, float pwm_phase);

#endif
