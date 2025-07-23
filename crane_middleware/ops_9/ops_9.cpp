#include "ops_9.hpp"

#include <cstring>

namespace crane
{

Ops_9::Ops_9(UART_HandleTypeDef * huart, bool use_dma) : huart(huart), use_dma_(use_dma) {}

void Ops_9::request()
{
  if (use_dma_) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf_, sizeof(rx_buf_));
  }
  else {
    HAL_UARTEx_ReceiveToIdle_IT(huart, rx_buf_, sizeof(rx_buf_));
  }
}
void Ops_9::update(const uint8_t * frame, uint16_t len)
{
  /* 1. 长度 & 帧头帧尾校验 */
  if (len != 28) return;
  if (frame[0] != 0x0D || frame[1] != 0x0A || frame[26] != 0x0A || frame[27] != 0x0D) return;

  /* 2. 拷贝 24 字节数据区 */
  memcpy(posture_.data, &frame[2], 24);

  /* 3. 解析 */
  parse_frame();
}

void Ops_9::parse_frame()
{
  zangle_ = posture_.val[0];
  xangle_ = posture_.val[1];
  yangle_ = posture_.val[2];
  pos_x_ = posture_.val[3];
  pos_y_ = posture_.val[4];
  w_z_ = posture_.val[5];

  if (zangle_ < -135.0f) zangle_ += 360.0f;
}

void Ops_9::send(const char * data, uint8_t len)
{
  if (use_dma_) {
    HAL_UART_Transmit_DMA(huart, reinterpret_cast<const uint8_t *>(data), len);
  }
  else {
    HAL_UART_Transmit(huart, reinterpret_cast<const uint8_t *>(data), len, 0xFF);
  }
}

void Ops_9::clear_zero() { send("ACT0", 4); }

void Ops_9::update_yaw(float angle)
{
  char buf[8] = "ACTJ";
  std::memcpy(buf + 4, &angle, 4);
  send(buf, 8);
}

void Ops_9::update_x(float x)
{
  char buf[8] = "ACTX";
  std::memcpy(buf + 4, &x, 4);
  send(buf, 8);
}

void Ops_9::update_y(float y)
{
  char buf[8] = "ACTY";
  std::memcpy(buf + 4, &y, 4);
  send(buf, 8);
}

void Ops_9::update_xy(float x, float y)
{
  char buf[12] = "ACTD";
  std::memcpy(buf + 4, &x, 4);
  std::memcpy(buf + 8, &y, 4);
  send(buf, 12);
}

float Ops_9::get_yaw() const { return zangle_; }
float Ops_9::get_x() const { return pos_x_; }
float Ops_9::get_y() const { return pos_y_; }
float Ops_9::get_wz() const { return w_z_; }

}  // namespace crane
