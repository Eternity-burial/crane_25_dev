// applications/host_serial.cpp
#include <cstdint>
#include <cstring>

extern "C" {
#include "cmsis_os.h"
#include "usart.h"
}

#include "motor_para.hpp"  // coordinate_t coordinate_data
extern coordinate_t coordinate_data;

extern "C" {
#include "sts3215.h"  // STS_SetAngle / STS_ReadAngle
}

// ---- 切换要使用的串口句柄：现在用 USART1 ----
#define HS_UART huart1

// ===================== 协议常量 =====================
static constexpr uint8_t STX0 = 0xAA;
static constexpr uint8_t STX1 = 0x55;
static constexpr uint8_t VER = 0x01;

enum : uint8_t
{
  CMD_PING = 0x01,
  CMD_SET_BLDC_POS = 0x10,
  CMD_SET_SERVO_POS = 0x11,
  CMD_GET_TELEM = 0x20,

  RSP_PONG = 0x81,
  RSP_ACK_BLDC = 0x90,
  RSP_ACK_SERVO = 0x91,
  RSP_TELEM = 0xA0,
};

// ===================== CRC16-CCITT ==================
static uint16_t crc16_ccitt(const uint8_t * p, uint16_t n)
{
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// ===================== 发送帧工具 ====================
static void uart_write(const uint8_t * buf, uint16_t len)
{
  HAL_UART_Transmit(&HS_UART, const_cast<uint8_t *>(buf), len, 100);
}

static void send_frame(uint8_t cmd, const uint8_t * payload, uint8_t len)
{
  // body = [VER, CMD, LEN, PAY...]
  uint8_t body[3 + 255];
  body[0] = VER;
  body[1] = cmd;
  body[2] = len;
  if (len && payload) memcpy(&body[3], payload, len);

  uint16_t crc = crc16_ccitt(body, (uint16_t)(3 + len));

  // out = [STX0, STX1, body..., CRC_L, CRC_H]
  uint8_t out[5 + 255 + 2];
  out[0] = STX0;
  out[1] = STX1;
  memcpy(&out[2], body, (size_t)(3 + len));
  out[5 + len] = (uint8_t)(crc & 0xFF);
  out[6 + len] = (uint8_t)(crc >> 8);

  uart_write(out, (uint16_t)(7 + len));
}

static void send_ack(uint8_t which, uint8_t status = 0)
{
  uint8_t p[1] = {status};
  send_frame(which, p, 1);
}

static void send_pong(const uint8_t * data, uint8_t len) { send_frame(RSP_PONG, data, len); }

static void send_telem()
{
  float telem[6] = {
    coordinate_data.x, coordinate_data.y, coordinate_data.zf, coordinate_data.zr, 0.0f, 0.0f};
  // 需要实测舵机角度时可解开以下两行（注意串口阻塞）
  // telem[4] = STS_ReadAngle(1);
  // telem[5] = STS_ReadAngle(2);
  send_frame(RSP_TELEM, reinterpret_cast<uint8_t *>(telem), sizeof(telem));
}

// ===================== 解析器 ========================
static uint8_t rx_byte;
static void arm_rx_it() { HAL_UART_Receive_IT(&HS_UART, &rx_byte, 1); }

static struct
{
  enum
  {
    S0,
    S1,
    S_VER,
    S_CMD,
    S_LEN,
    S_PAY,
    S_CRC0,
    S_CRC1
  } st = S0;
  uint8_t ver = 0, cmd = 0, len = 0, pay[255];
  uint16_t pay_pos = 0;
  uint16_t crc_recv = 0;
} P;

static void parser_reset() { P = {}; }

static void parser_feed(uint8_t b)
{
  switch (P.st) {
    case P.S0:
      P.st = (b == STX0) ? P.S1 : P.S0;
      break;
    case P.S1:
      if (b == STX1)
        P.st = P.S_VER;
      else
        P.st = P.S0;
      break;
    case P.S_VER:
      P.ver = b;
      P.st = P.S_CMD;
      break;
    case P.S_CMD:
      P.cmd = b;
      P.st = P.S_LEN;
      break;
    case P.S_LEN:
      P.len = b;
      P.pay_pos = 0;
      P.st = (P.len ? P.S_PAY : P.S_CRC0);
      break;
    case P.S_PAY:
      P.pay[P.pay_pos++] = b;
      if (P.pay_pos >= P.len) P.st = P.S_CRC0;
      break;
    case P.S_CRC0:
      P.crc_recv = b;
      P.st = P.S_CRC1;
      break;
    case P.S_CRC1: {
      P.crc_recv |= (uint16_t)b << 8;

      uint8_t body[3 + 255];
      body[0] = P.ver;
      body[1] = P.cmd;
      body[2] = P.len;
      if (P.len) memcpy(&body[3], P.pay, P.len);
      uint16_t crc = crc16_ccitt(body, (uint16_t)(3 + P.len));

      if (crc == P.crc_recv && P.ver == VER) {
        // ======= 执行命令 =======
        if (P.cmd == CMD_PING) {
          send_pong(P.pay, P.len);
        }
        else if (P.cmd == CMD_SET_BLDC_POS && P.len == 1 + 4) {
          uint8_t axis = P.pay[0];
          float val;
          memcpy(&val, &P.pay[1], 4);
          switch (axis) {
            case 0:
              coordinate_data.x = val;
              break;
            case 1:
              coordinate_data.y = val;
              break;
            case 2:
              coordinate_data.zf = val;
              break;
            case 3:
              coordinate_data.zr = val;
              break;
            default:
              break;
          }
          send_ack(RSP_ACK_BLDC, 0);
        }
        else if (P.cmd == CMD_SET_SERVO_POS && P.len == 1 + 4 + 4 + 4) {
          uint8_t id = P.pay[0];
          float deg, spd, acc;
          memcpy(&deg, &P.pay[1], 4);
          memcpy(&spd, &P.pay[5], 4);
          memcpy(&acc, &P.pay[9], 4);
          if (id == 1 || id == 2) {
            STS_SetAngle(id, deg, spd, acc);
            send_ack(RSP_ACK_SERVO, 0);
          }
          else {
            send_ack(RSP_ACK_SERVO, 1);
          }
        }
        else if (P.cmd == CMD_GET_TELEM) {
          send_telem();
        }
      }
      parser_reset();
    } break;
  }
}

// ========= HAL 回调（中断仅收一字节，解析放在此） =========
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart == &HS_UART) {
    parser_feed(rx_byte);
    arm_rx_it();
  }
}

// ===================== 任务入口 ======================
extern "C" void host_serial_task(void const * arg)
{
  osDelay(100);
  parser_reset();
  arm_rx_it();
  const char banner[] = "HS v1 ready (USART1)\r\n";
  uart_write(reinterpret_cast<const uint8_t *>(banner), sizeof(banner) - 1);

  for (;;) {
    // 可在此做周期上报
    osDelay(10);
  }
}
