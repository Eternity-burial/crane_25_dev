#ifndef OPS__9_HPP_
#define OPS__9_HPP_

#include <cstdint>

#include "usart.h"  // HAL, USART_HandleTypeDef

namespace crane
{

class Ops_9
{
public:
  Ops_9(UART_HandleTypeDef * huart, bool use_dma = true);

  UART_HandleTypeDef * huart;
  uint8_t rx_buf_[28];

  void request();                                    // 启动 DMA/IT 接收
  void update(const uint8_t * frame, uint16_t len);  // 用于中断调用（按字节处理）

  void send(const char * data, uint8_t len);
  void clear_zero();
  void update_yaw(float angle);
  void update_x(float x);
  void update_y(float y);
  void update_xy(float x, float y);

  float get_yaw() const;
  float get_x() const;
  float get_y() const;
  float get_wz() const;

private:
  void parse_frame();

  bool use_dma_;
  HAL_StatusTypeDef hal_status_;

  uint8_t tx_buf_[256];
  uint8_t tx_count_;
  uint8_t count_;
  uint8_t state_;

  union {
    uint8_t data[24];
    float val[6];  // zangle, xangle, yangle, pos_x, pos_y, w_z
  } posture_;

  float zangle_, xangle_, yangle_, pos_x_, pos_y_, w_z_;
};

}  // namespace crane

#endif  // OPS__9_HPP_