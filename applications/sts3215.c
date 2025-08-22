// applications/sts3215.c
// 与工程独立，只依赖 sts3215.h / sts_port.h
#include "sts3215.h"
#include "sts_port.h"
#include <string.h>
#include <math.h>

#define STS_HDR 0xFF

// ================= 工具 =================
static inline uint8_t csum(uint8_t id, uint8_t len, uint8_t inst, const uint8_t* p, uint8_t n){
    uint16_t s = id + len + inst;
    for (uint8_t i=0;i<n;i++) s += p[i];
    return (uint8_t)(~s);
}

static inline uint16_t deg2raw(float deg){
    while (deg < 0.0f)    deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    const float r = (float)STS_POS_MAX_RAW / STS_POS_MAX_DEG;
    int v = (int)lroundf(deg * r);
    if (v < 0) v = 0;
    if (v > STS_POS_MAX_RAW) v = STS_POS_MAX_RAW;
    return (uint16_t)v;
}
static inline float raw2deg(uint16_t raw){
    return (float)raw * (STS_POS_MAX_DEG / (float)STS_POS_MAX_RAW);
}
static inline uint16_t dps2raw(float dps){
    if (dps <= 0.0f) return 0; // 0=默认
    return (uint16_t)lroundf(dps / STS_SPD_UNIT_DPS);
}
static inline uint8_t dpss2raw(float dps2){
    if (dps2 <= 0.0f) return 0;
    int v = (int)lroundf(dps2 / STS_ACC_UNIT_DPS2);
    if (v < 1) v = 1;
    if (v > 254) v = 254;
    return (uint8_t)v;
}

// ================= 原始读写 =================
int STS_Write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t bytes){
    // FF FF ID LEN INST=WRITE ADDR [DATA...] CHK
    uint8_t buf[8 + 32];
    uint8_t len = (uint8_t)(3 + bytes);      // INST + ADDR + DATA...
    uint8_t idx = 0;
    buf[idx++]=STS_HDR; buf[idx++]=STS_HDR; buf[idx++]=id; buf[idx++]=len;
    buf[idx++]=STS_INST_WRITE; buf[idx++]=addr;
    memcpy(&buf[idx], data, bytes); idx += bytes;
    buf[idx++] = csum(id, len, STS_INST_WRITE, &buf[5], (uint8_t)(1+bytes));
    return (STS_Port_Tx(buf, idx, 50) == (int)idx) ? 1 : 0;
}

// 可用于严格同步：先把数据写到寄存器缓存（不立即执行），再 ACTION 触发
static int STS_RegWrite(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t bytes){
    uint8_t buf[8 + 32];
    uint8_t len = (uint8_t)(3 + bytes);
    uint8_t idx = 0;
    buf[idx++]=STS_HDR; buf[idx++]=STS_HDR; buf[idx++]=id; buf[idx++]=len;
    buf[idx++]=STS_INST_REG_WRITE; buf[idx++]=addr;
    memcpy(&buf[idx], data, bytes); idx += bytes;
    buf[idx++] = csum(id, len, STS_INST_REG_WRITE, &buf[5], (uint8_t)(1+bytes));
    return (STS_Port_Tx(buf, idx, 50) == (int)idx) ? 1 : 0;
}

static int STS_Action(void){
    uint8_t id  = 0xFE;          // 广播
    uint8_t len = 0x02;          // 仅 INST
    uint8_t buf[7];
    buf[0]=STS_HDR; buf[1]=STS_HDR; buf[2]=id; buf[3]=len; buf[4]=STS_INST_ACTION;
    buf[5]=csum(id, len, STS_INST_ACTION, NULL, 0);
    return (STS_Port_Tx(buf, 6, 50) == 6) ? 1 : 0;
}

int STS_Read(uint8_t id, uint8_t addr, uint8_t* data, uint8_t bytes){
    // 主机：FF FF ID LEN INST=READ ADDR SIZE CHK
    uint8_t req[8];
    uint8_t len = 0x04; // INST + ADDR + SIZE
    req[0]=STS_HDR; req[1]=STS_HDR; req[2]=id; req[3]=len;
    req[4]=STS_INST_READ; req[5]=addr; req[6]=bytes;
    req[7]=csum(id, len, STS_INST_READ, &req[5], 2);
    if (STS_Port_Tx(req, 8, 50) != 8) return 0;

    // 从机：FF FF ID LEN INST DATA... CHK
    uint8_t hdr[4];
    if (STS_Port_Rx(hdr, 4, 50) != 4) return 0;
    if (!(hdr[0]==STS_HDR && hdr[1]==STS_HDR)) return 0;
    uint8_t rid=hdr[2], rlen=hdr[3];
    if (rid != id) return 0;

    // rlen字节（INST + DATA...）+ CHK
    uint8_t rest[2 + 255];
    if (STS_Port_Rx(rest, rlen + 1, 50) != (int)rlen + 1) return 0;

    // 校验和
    {
        uint8_t calc = csum(rid, rlen, rest[0], &rest[1], (uint8_t)(rlen - 1));
        if (calc != rest[rlen]) return 0;
    }

    if (rlen < (uint8_t)(1 + bytes)) return 0; // 至少 INST(1)+DATA(bytes)
    memcpy(data, &rest[1], bytes);
    return 1;
}

// ================= 高层控制 =================
int STS_Torque(uint8_t id, int enable){
    uint8_t v = enable ? 1 : 0;
    return STS_Write(id, STS_ADDR_TORQUE_EN, &v, 1);
}

// 单机到角度（可选速度/加速度）
int STS_SetAngle(uint8_t id, float deg, float speed_dps, float acc_dps2){
    if (!STS_Torque(id, 1)) return 0;

    // 可选：加速度（0 跳过）
    uint8_t acc = dpss2raw(acc_dps2);
    if (acc){
        if (!STS_Write(id, STS_ADDR_GOAL_ACC, &acc, 1)) return 0;
    }

    // 速度（0 跳过）；目标块总是写全（pos/time/speed）
    uint16_t spd = dps2raw(speed_dps);
    uint16_t pos = deg2raw(deg);
    uint8_t  blk[6] = {
        (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
        0x00, 0x00,                            // Time=0（走速度曲线）
        (uint8_t)(spd & 0xFF), (uint8_t)(spd >> 8)
    };
    return STS_Write(id, STS_ADDR_GOAL_BLOCK, blk, 6);
}

// 保留：SYNC_WRITE 写同一地址区到多个ID（本实现用于写6字节目标块）
static int STS_SyncWrite(uint8_t start_addr, uint8_t bytes_per_id,
                         const uint8_t* id_and_payload, uint8_t id_count)
{
    // FF FF FE LEN INST=SYNC_WRITE START_ADDR BYTES [ID DATA...]*N CHK
    uint8_t buf[96];
    uint8_t payload_len = (uint8_t)(2 + id_count * (1 + bytes_per_id));
    uint8_t len = (uint8_t)(2 + payload_len);  // INST + payload
    uint8_t idx=0;
    buf[idx++]=STS_HDR; buf[idx++]=STS_HDR; buf[idx++]=0xFE; buf[idx++]=len;
    buf[idx++]=STS_INST_SYNC_WRITE; buf[idx++]=start_addr; buf[idx++]=bytes_per_id;
    memcpy(&buf[idx], id_and_payload, payload_len - 2);
    idx += (payload_len - 2);
    buf[idx++] = csum(0xFE, len, STS_INST_SYNC_WRITE, &buf[5], payload_len);
    return (STS_Port_Tx(buf, idx, 50) == (int)idx) ? 1 : 0;
}

// 双机同步角度（严格同时）：RegWrite 两台，再 Action 一次触发
int STS_SetAngleSync(uint8_t id1, float deg1, uint8_t id2, float deg2,
                     float speed_dps, float acc_dps2)
{
    if (!STS_Torque(id1,1) || !STS_Torque(id2,1)) return 0;

    uint8_t  acc = dpss2raw(acc_dps2);
    uint16_t spd = dps2raw(speed_dps);
    uint16_t p1  = deg2raw(deg1);
    uint16_t p2  = deg2raw(deg2);

    // 1) 可选：加速度
    if (acc){
        if (!STS_RegWrite(id1, STS_ADDR_GOAL_ACC, &acc, 1)) return 0;
        if (!STS_RegWrite(id2, STS_ADDR_GOAL_ACC, &acc, 1)) return 0;
    }

    // 2) 目标块：Pos[2], Time[2]=0, Speed[2]
    uint8_t blk1[6] = {
        (uint8_t)(p1 & 0xFF), (uint8_t)(p1 >> 8),
        0x00, 0x00,
        (uint8_t)(spd & 0xFF), (uint8_t)(spd >> 8)
    };
    uint8_t blk2[6] = {
        (uint8_t)(p2 & 0xFF), (uint8_t)(p2 >> 8),
        0x00, 0x00,
        (uint8_t)(spd & 0xFF), (uint8_t)(spd >> 8)
    };
    if (!STS_RegWrite(id1, STS_ADDR_GOAL_BLOCK, blk1, 6)) return 0;
    if (!STS_RegWrite(id2, STS_ADDR_GOAL_BLOCK, blk2, 6)) return 0;

    // 3) 一次 ACTION 同步触发
    return STS_Action();
}

float STS_ReadAngle(uint8_t id){
    uint8_t d[2];
    if (!STS_Read(id, STS_ADDR_PRESENT_POS, d, 2)) return -1.0f;
    uint16_t raw = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
    return raw2deg(raw);
}
