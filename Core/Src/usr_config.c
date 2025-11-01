#include "usr_config.h"
#include "util.h"
#include "flash.h"
#include "string.h"

tUsrConfig UsrConfig;

#define FMC_PAGE_SIZE ((uint16_t)0x800U)                                  // 2KB
#define USR_CONFIG_ROM_ADDR ((uint32_t)(0x8000000 + 60 * FMC_PAGE_SIZE)) // Page 60

void USR_CONFIG_set_default_config(void)
{
    // Motor
    UsrConfig.motor_pole_pairs = 7;
		UsrConfig.motor_phase_resistance = 0.160473f;
		UsrConfig.motor_phase_inductance = 0.000028f;
    UsrConfig.inertia = 0.001f;

    // Calib
    UsrConfig.calib_valid = 0;
    UsrConfig.calib_current = 1.2f;
    UsrConfig.calib_max_voltage = 10.0f;

    // Control
    UsrConfig.control_mode = CONTROL_MODE_POSITION_TRAP;
    UsrConfig.current_ramp_rate = 2.0f;
    UsrConfig.vel_ramp_rate = 50.0f;
    UsrConfig.traj_vel = 50;
    UsrConfig.traj_accel = 100;
    UsrConfig.traj_decel = 100;
    UsrConfig.pos_gain = 160.0f;
    UsrConfig.vel_gain = 0.8f;
    UsrConfig.vel_integrator_gain = 0.08f;
    UsrConfig.vel_limit = 90;
    UsrConfig.current_limit = 20;
    UsrConfig.current_ctrl_bandwidth = 1000;

    // Protect
    UsrConfig.protect_under_voltage = 11.2f;
    UsrConfig.protect_over_voltage = 40;
    UsrConfig.protect_over_speed = 100;

    // Can
    UsrConfig.can_id = 55;
    UsrConfig.can_timeout_ms = 0;
    UsrConfig.can_sync_target_enable = 0;
		
	// MIT
	UsrConfig.pos_max = 0;
	UsrConfig.vel_max = 0;
	UsrConfig.iq_max = 0;

}

int USR_CONFIG_read_config(void)
{
	int state = 0;

	memcpy(&UsrConfig, (uint8_t*)USR_CONFIG_ROM_ADDR, sizeof(tUsrConfig));

	uint32_t crc;
	crc = crc32((uint8_t*)&UsrConfig, (uint8_t*)&UsrConfig.crc - (uint8_t*)&UsrConfig, 0);
	if(crc != UsrConfig.crc){
		state = -1;
	}

    // CAN ID 限制0~15
	if(UsrConfig.can_id > 0xF){UsrConfig.can_id = 0xF;}
	
	return state;
}

int USR_CONFIG_save_config(void)
{
	int status = 0;
	
	uint32_t primask = cpu_enter_critical();
	
    FLASH_unlock();
	
	// Erase
	status = FLASH_erase_page(60, 1);

	// 64位写入忽略问题修正
	if(status == 0){
    UsrConfig.crc = crc32((uint8_t*)&UsrConfig, sizeof(tUsrConfig)-4, 0);
    uint8_t* pData = (uint8_t*)&UsrConfig;
    for(int i=0; i<sizeof(tUsrConfig); i+=8){
        uint64_t data = 0xFFFFFFFFFFFFFFFF;
        int remain = sizeof(tUsrConfig) - i;
        if(remain >= 8){
            memcpy(&data, pData + i, 8);
        }else{
            memcpy(&data, pData + i, remain);
        }
        status = FLASH_program(USR_CONFIG_ROM_ADDR + i, data);
        if(status != 0){
					DEBUG("FLASH_program Error\n\r");
            break;
        }
    }
	}
	else{DEBUG("FLASH_erase_page Error\n\r");}
	
	FLASH_lock();
	
	cpu_exit_critical(primask);
	
	return status;
}
