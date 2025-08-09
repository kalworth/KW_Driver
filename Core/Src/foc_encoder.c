#include "foc_encoder.h"
#include "usr_config.h"
#include "spi.h"
#include "util.h"

#define cs_down LL_GPIO_ResetOutputPin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin)
#define cs_up		LL_GPIO_SetOutputPin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin)

tEncoder Encoder;
static int spi_transmit_receive(uint16_t data_in, uint16_t *data_out) {
    int state = 0;
    uint32_t timeout_cnt;
    static const uint32_t timeout_cnt_num = 10000;

    // 1. 等待发送缓冲区空（TXE）
    timeout_cnt = 0;
    while (!LL_SPI_IsActiveFlag_TXE(SPI2)) {
        if (++timeout_cnt > timeout_cnt_num) {
            state = -1;
            goto error;
        }
    }

    // 2. 发送数据
    LL_SPI_TransmitData16(SPI2, data_in);

    // 3. 等待接收完成（RXNE） - 关键修改点！
    timeout_cnt = 0;
    while (!LL_SPI_IsActiveFlag_RXNE(SPI2)) {
        if (++timeout_cnt > timeout_cnt_num) {
            state = -1;
            goto error;
        }
    }

    // 4. 读取接收数据
    *data_out = LL_SPI_ReceiveData16(SPI2);

    // 5. 等待总线空闲（BSY） - 确保传输完全结束
    timeout_cnt = 0;
    while (LL_SPI_IsActiveFlag_BSY(SPI2)) {
        if (++timeout_cnt > timeout_cnt_num) {
            state = -1;
            goto error;
        }
    }

    return state;

error:
    // 可选：添加SPI错误恢复（如复位外设）
    // LL_SPI_Disable(SPI1);
    // ... 复位操作 ...
    // LL_SPI_Enable(SPI1);
    return state;
}

uint16_t Encoder_read(void)
{
	uint16_t txData = 0;
	uint16_t rxData;

	// CS
	LL_GPIO_ResetOutputPin(SPI2_CSN_GPIO_Port, SPI2_CSN_Pin);
	
	spi_transmit_receive(txData, &rxData);
	
	// NCS
	LL_GPIO_SetOutputPin(SPI2_CSN_GPIO_Port, SPI2_CSN_Pin);


	return (rxData>>2);
}

void ENCODER_sample(float dt)
{
    if (UsrConfig.calib_valid)
    {
        // dir swap
        if (UsrConfig.encoder_dir_rev)
        {
            Encoder.raw = ENCODER_CPR - Encoder_read();
        }
        else
        {
            Encoder.raw = Encoder_read();
        }

        // offset compensation
        int off_1 = UsrConfig.offset_lut[Encoder.raw >> 7];
        int off_2 = UsrConfig.offset_lut[((Encoder.raw >> 7) + 1) % 128];
        int off_interp = off_1 + ((off_2 - off_1) * (Encoder.raw - ((Encoder.raw >> 7) << 7)) >> 7); // Interpolate between lookup table entries
        int cnt = Encoder.raw - off_interp;                                                          // Correct for nonlinearity with lookup table from calibration
        if (cnt > ENCODER_CPR)
        {
            cnt -= ENCODER_CPR;
        }
        else if (cnt < 0)
        {
            cnt += ENCODER_CPR;
        }
        Encoder.cnt = cnt;
    }
    else
    {
        // dir swap
        if (UsrConfig.encoder_dir_rev)
        {
            Encoder.raw = ENCODER_CPR - Encoder_read();
        }
        else
        {
            Encoder.raw = Encoder_read();
        }
        Encoder.cnt = Encoder.raw;
    }

    // Turns
    if (Encoder.cnt - Encoder.cnt_last > ENCODER_CPR / 2)
    {
        Encoder.turns -= 1;
    }
    else if (Encoder.cnt - Encoder.cnt_last < -ENCODER_CPR / 2)
    {
        Encoder.turns += 1;
    }
    Encoder.cnt_last = Encoder.cnt;

    // Position
    Encoder.position = Encoder.turns + (float)Encoder.cnt / (float)ENCODER_CPR;
		Encoder.position_output = fmodf_pos(Encoder.position, 1.0f) * M_2PI;
		
    // Elec angle
    Encoder.elec_angle = (UsrConfig.motor_pole_pairs * (float)(Encoder.cnt - UsrConfig.encoder_offset)) / ((float)ENCODER_CPR);
    Encoder.elec_angle = 2.0f * M_PI * (Encoder.elec_angle - (int)Encoder.elec_angle);
    Encoder.elec_angle = Encoder.elec_angle < 0 ? Encoder.elec_angle + 2.0f * M_PI : Encoder.elec_angle;

    float vel = (Encoder.position - Encoder.position_last) / dt;
    Encoder.position_last = Encoder.position;

    float sum = vel;
    for (int i = 1; i < VEL_VEC_SIZE; i++)
    {
        Encoder.vel_vec[VEL_VEC_SIZE - i] = Encoder.vel_vec[VEL_VEC_SIZE - i - 1];
        sum += Encoder.vel_vec[VEL_VEC_SIZE - i];
    }
    Encoder.vel_vec[0] = vel;

    Encoder.velocity = sum / ((float)VEL_VEC_SIZE);
    Encoder.velocity_output = Encoder.velocity * 2.0f * M_PI; 
    Encoder.velocity_elec = 2.0f * M_PI * Encoder.velocity * UsrConfig.motor_pole_pairs;
}



