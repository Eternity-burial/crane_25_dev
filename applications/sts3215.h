// sts3215.h —— STS3215 总线舵机：独立、可迁移的最小控制层
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// —— 指令码（飞特协议）
#define STS_INST_PING         0x01
#define STS_INST_READ         0x02
#define STS_INST_WRITE        0x03
#define STS_INST_REG_WRITE    0x04
#define STS_INST_ACTION       0x05
#define STS_INST_SYNC_WRITE   0x83

// —— 常用内存表（STS/SMS 磁编码系列通用，若与你手册不同请改这里）
#define STS_ADDR_TORQUE_EN    0x28    // 1字节：0关/1开
#define STS_ADDR_GOAL_ACC     0x29    // 1字节：1~254，0忽略
#define STS_ADDR_GOAL_BLOCK   0x2A    // 连续6字节：Pos[2], Time[2], Speed[2]
#define STS_ADDR_PRESENT_POS  0x38    // 2字节：当前位置（原始）

// —— 角度/速度映射（STS3215 为磁编码 0~4095≈0~360°，可按手册修正）
#define STS_POS_MAX_DEG       360.0f
#define STS_POS_MAX_RAW       4095
// 速度/加速度单位（若不确定，先设 1raw≈1°/s；ACC 1~254）
#define STS_SPD_UNIT_DPS      1.0f
#define STS_ACC_UNIT_DPS2     1.0f

// —— 对外 API ——
// 单机到角度(°)，可选速度(°/s)/加速度(°/s^2)；speed/acc=0 表示默认/忽略
int   STS_SetAngle(uint8_t id, float deg, float speed_dps, float acc_dps2);

// 双机同步到角度（同启），共用同一速度、加速度
int   STS_SetAngleSync(uint8_t id1, float deg1, uint8_t id2, float deg2,
                       float speed_dps, float acc_dps2);

// 读当前位置（角度）；返回 >=0 为角度，<0 失败
float STS_ReadAngle(uint8_t id);

// 扭矩开/关（1开/0关）
int   STS_Torque(uint8_t id, int enable);

// 原始寄存器读写（可扩展用）
int   STS_Write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t bytes);
int   STS_Read (uint8_t id, uint8_t addr,       uint8_t* data, uint8_t bytes);

#ifdef __cplusplus
}
#endif
