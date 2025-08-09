#include "led.h"
#include "fsm.h"
#include "gpio.h"
#include "systick.h"

static inline void LED_set(uint32_t state){
	if(state){
		LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);
	}else{
		LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
	}
}

void LED_loop(void)
{
	static uint32_t tick = 0;
	static uint16_t cnt = 0;
	
	// 1000Hz
	if(SYSTICK_get_ms_since(tick) < 1){
		return;
	}
	tick = SYSTICK_get_tick();
	
	switch(FSM_get_stat()){
		case FS_MENU_MODE:
			if(cnt == 0){
				LED_set(1);
			}else if(cnt == 100){
				LED_set(0);
			}else if(cnt >= 1500){
				cnt = 0xFFFF;
			}
			break;
		
		case FS_MOTOR_MODE:
			if(cnt == 0){
				LED_set(1);
			}else if(cnt == 100){
				LED_set(0);
			}else if(cnt == 200){
				LED_set(1);
			}else if(cnt == 300){
				LED_set(0);
			}else if(cnt >= 1500){
				cnt = 0xFFFF;
			}
			break;
		
		case FS_CALIBRATION_MODE:
			if(cnt == 0){
				LED_set(1);
			}else if(cnt == 100){
				LED_set(0);
			}else if(cnt == 200){
				LED_set(1);
			}else if(cnt == 300){
				LED_set(0);
			}else if(cnt == 400){
				LED_set(1);
			}else if(cnt == 500){
				LED_set(0);
			}else if(cnt >= 1500){
				cnt = 0xFFFF;
			}
			break;
		
		case FS_UART_SETUP:
			LED_set(1);
			break;
		
		default:
			break;
	}
	
	cnt ++;
}
