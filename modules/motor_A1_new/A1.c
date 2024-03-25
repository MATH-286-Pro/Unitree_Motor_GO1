#include "usart.h"
#include "A1.h"
#include "crc_ccitt.h"
#include "stdio.h"

#define PI 3.1415926f

// 限幅函数
#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 


//uint32_t crc32_core(uint32_t* ptr, uint32_t len)
//{
//    uint32_t xbit = 0;
//    uint32_t data = 0;
//    uint32_t CRC32 = 0xFFFFFFFF;
//    const uint32_t dwPolynomial = 0x04c11db7;
//    for (uint32_t i = 0; i < len; i++)
//    {
//        xbit = 1 << 31;
//        data = ptr[i];
//        for (uint32_t bits = 0; bits < 32; bits++)
//        {
//            if (CRC32 & 0x80000000)
//            {
//                CRC32 <<= 1;
//                CRC32 ^= dwPolynomial;
//            }
//            else
//                CRC32 <<= 1;
//            if (data & xbit)
//                CRC32 ^= dwPolynomial;

//            xbit >>= 1;
//        }
//    }
//    return CRC32;
//}


// 数据发送处理函数
int modify_data_A1(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;

    // 最大最小限制 A1
    SATURATE(motor_s->K_P,  0.0f,   16f);
    SATURATE(motor_s->K_W,  0.0f,   32f);
    SATURATE(motor_s->T,   -128f,  128f);
    SATURATE(motor_s->W,   -256f,  256f);
    SATURATE(motor_s->Pos, -823549f,  823549f);

    // 发送数据处理 A1
    motor_s->motor_send_data.mode.id       = motor_s->id;
    motor_s->motor_send_data.mode.status   = motor_s->mode;
    motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
    motor_s->motor_send_data.comd.spd_des  = motor_s->W*128;
    motor_s->motor_send_data.comd.pos_des  = motor_s->Pos*16384/PI;
    motor_s->motor_send_data.comd.k_pos    = motor_s->K_P/25.6f*32768;
    motor_s->motor_send_data.comd.k_spd    = motor_s->K_W/25.6f*32768;

    // 校验？
    motor_s->motor_send_data.CRC16         = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}


// 数据接收处理函数
int extract_data_A1(MOTOR_recv *motor_r)
{
    if(motor_r->motor_recv_data.CRC16 !=
        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
        // printf("[WARNING] Receive data CRC error");
        motor_r->correct = 0;
        return motor_r->correct;
    }
    else
		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
        motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
        motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
		motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		motor_r->correct = 1;

        return motor_r->correct;
    }
}

// 发送函数 
void motor_A1_send()
{
    HAL_UART_Transmit_IT(&huart1,Data_Box[ID],34);               //这个是用UART
}



// 发送 + 接收 函数
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)
{
    uint16_t rxlen = 0;
		
	
		// 数据处理 !!!
    modify_data_A1(pData);
    
		// // 转换板 发送使能
		// SET_485_DE_UP();
		// SET_485_RE_UP();
    // HAL_UART_Transmit(&huart1, (uint8_t *)pData, sizeof(pData->motor_send_data), 10); 
		
		// // 转换板 接收使能
		// SET_485_RE_DOWN();
		// SET_485_DE_DOWN();
    // HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rData, sizeof(rData->motor_recv_data), &rxlen, 10);
		

    if(rxlen == 0)

      return HAL_TIMEOUT;

    if(rxlen != sizeof(rData->motor_recv_data))
			return HAL_ERROR;

    uint8_t *rp = (uint8_t *)&rData->motor_recv_data;
    if(rp[0] == 0xFE && rp[1] == 0xEE)
    {
        rData->correct = 1;
        extract_data_A1(rData);
        return HAL_OK;
    }
    
    return HAL_ERROR;
}
