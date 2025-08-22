// applications/sts_port_hal.c
#include "sts_port.h"
#include "usart.h"
#include "main.h"

// ===== 选择与你舵机总线相连的串口句柄 =====
extern UART_HandleTypeDef huart6;   // 如果你用的是别的串口，改成对应的 extern
#define STS_UART   (&huart6)

// 发送 len 字节，返回已发送字节数；timeout_ms 为阻塞上限
int STS_Port_Tx(const uint8_t* data, uint16_t len, uint32_t timeout_ms){
    return (HAL_UART_Transmit(STS_UART, (uint8_t*)data, len, timeout_ms) == HAL_OK) ? len : 0;
}

// 接收 len 字节，返回实际接收数；timeout_ms 为阻塞上限（收不到可返回 0）
int STS_Port_Rx(uint8_t* data, uint16_t len, uint32_t timeout_ms){
    return (HAL_UART_Receive (STS_UART, data, len, timeout_ms) == HAL_OK) ? len : 0;
}

// 毫秒级延时（发命令或等待应答时用）
void STS_Port_DelayMs(uint32_t ms){
    HAL_Delay(ms);
}
