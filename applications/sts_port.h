// sts_port.h —— 平台移植层：你只需在工程里提供这3个函数的实现
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 发送 len 字节，返回已发送字节数；timeout_ms 为阻塞上限
int STS_Port_Tx(const uint8_t* data, uint16_t len, uint32_t timeout_ms);

// 接收 len 字节，返回实际接收数；timeout_ms 为阻塞上限（收不到可返回 0）
int STS_Port_Rx(uint8_t* data, uint16_t len, uint32_t timeout_ms);

// 毫秒级延时（发命令或等待应答时用）
void STS_Port_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif
