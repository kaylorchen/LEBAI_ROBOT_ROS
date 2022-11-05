//
// Created by kaylor on 10/25/22.
//
#include "tt_driver.h"

#include <unistd.h>

__attribute__((weak)) void CanDelayMs(const uint32_t count) {}

__attribute__((weak)) void WriteData(const uint8_t number,
                                     const uint32_t can_id, const uint8_t *data,
                                     const uint8_t size) {}

__attribute__((weak)) void ReadData(const uint8_t number, uint32_t *can_id,
                                    uint8_t *data, uint8_t *size) {}

void SdoWrite(const uint8_t number, const uint32_t can_id, const uint8_t code,
              const uint16_t index, const uint8_t sub_index,
              const uint32_t data) {
  uint8_t buffer[8];
  buffer[0] = code;
  buffer[1] = index & 0x00FF;
  buffer[2] = index >> 8;
  buffer[3] = sub_index;
  *((uint32_t *)(&buffer[4])) = data;
  WriteData(number, can_id, buffer, 8);
  CanDelayMs(SDO_DELAY);
}

void SdoRead(const uint8_t number, uint32_t *can_id, uint8_t *code,
             uint16_t *index, uint8_t *sub_index, uint32_t *data) {
  uint8_t buffer[8];
  uint8_t size;
  ReadData(number, can_id, buffer, &size);
  *code = buffer[0];
  *index = *((uint16_t *)&buffer[1]);
  *sub_index = buffer[3];
  *data = *((uint32_t *)&buffer[4]);
}

void EnablePositionMode(const uint8_t number, const uint32_t can_id) {
  uint32_t read_can_id;
  uint8_t read_code;
  uint16_t read_index;
  uint8_t read_sub_index;
  uint32_t read_data;
#if DRIVER_TYPE == TT_DRIVER
#elif DRIVER_TYPE == ZL_DRIVER
  // 设置位置模式
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x6060, 0x00, 0x01);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置左电机 S形加速时间为100ms
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6083, 0x01, 0x64);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置右电机 S形加速时间为100ms
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6083, 0x02, 0x64);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置左电机 S形减速时间为100ms
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6084, 0x01, 0x64);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置右电机 S形减速时间为100ms
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6084, 0x02, 0x64);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置左电机 最大速度60rpm
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6081, 0x01, 0x3C);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置右电机 最大速度60rpm
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6081, 0x02, 0x3C);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 使能
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x06);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x07);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x0F);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
#endif
}

void EnableSpeedMode(const uint8_t number, const uint32_t can_id) {
  uint32_t read_can_id;
  uint8_t read_code;
  uint16_t read_index;
  uint8_t read_sub_index;
  uint32_t read_data;
#if DRIVER_TYPE == TT_DRIVER
  // 设置速度模式和反馈
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x6060, 0x00, 0x03);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 清除报警异常和反馈
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x0080);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 等待使能和反馈
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x0006);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 就绪和反馈
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x0007);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 使能和反馈
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x000F);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
#elif DRIVER_TYPE == ZL_DRIVER
  // 设置异步模式
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x200F, 0x00, 0x00);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置速度模式
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x6060, 0x00, 0x03);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置左电机加速时间为0ms
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6083, 0x01, 0x00);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置右电机加速时间为0ms
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6083, 0x02, 0x00);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置左电机减速时间为0ms
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6084, 0x01, 0x00);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置右电机减速速时间为0ms
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x6084, 0x02, 0x00);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 使能
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x06);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x07);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x0F);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
#endif
}

void EnableTpdo1(const uint8_t number, const uint32_t can_id) {
  uint32_t read_can_id;
  uint8_t read_code;
  uint16_t read_index;
  uint8_t read_sub_index;
  uint32_t read_data;
  uint32_t id = can_id - 0x600;
#if DRIVER_TYPE == TT_DRIVER
#elif DRIVER_TYPE == ZL_DRIVER
  // 禁止TPDO1
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1800, 0x01, 0x80000180 + id);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 清空TPDO1映射
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x1A00, 0x00, 0x00);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 把左反馈速度(32bits)（606C:01）映射到1A00：01
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1A00, 0x01, 0x606C0120);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 把左反馈速度(32bits)（606C:02）映射到1A00：02
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1A00, 0x02, 0x606C0220);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置TPDO1的子索引数目为2，手册中1A00：00
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x1A00, 0x00, 0x02);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置TPDO1的传输方式为定时器触发
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x1800, 0x02, 0xFF);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置定时器时间为 n ms
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x1800, 0x05, SAMPLING_PERIOD * 2);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 使能TPDO1
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1800, 0x01, 0x00000180 + id);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
#endif
}

void EnableRpdo1(const uint8_t number, const uint32_t can_id) {
  uint32_t read_can_id;
  uint8_t read_code;
  uint16_t read_index;
  uint8_t read_sub_index;
  uint32_t read_data;
  uint32_t id = can_id - 0x600;
#if DRIVER_TYPE == TT_DRIVER
  // 禁止PDO1
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1400, 0x01, 0x80000200 + id);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 配置为异步模式，收到数据立即更新
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x1400, 0x02, 0xFF);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 把目标速度(32bits)（60FF）映射到1600：01
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1600, 0x01, 0x60FF0020);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置1600的映射PDO的个数
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x1600, 0x00, 0x01);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 使能PDO1
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1400, 0x01, 0x00000200 + id);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
#elif DRIVER_TYPE == ZL_DRIVER
  // 1400的初始化的数值都是对的，不用再配置
  // 禁止RPDO1
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1400, 0x01, 0x80000200 + id);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 清空PDO1的子索引数目，手册中1600：00这个位置是只读，但是举例子中是可写的
  // 数据手册错了，这个可写
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x1600, 0x00, 0x00);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 把左目标速度(32bits)（60FF:01）映射到1600：01
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1600, 0x01, 0x60FF0120);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 把右目标速度(32bits)（60FF:02）映射到1600：02
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1600, 0x02, 0x60FF0220);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 设置PDO1的子索引数目为2，手册中1600：00 数据手册错了，这个可写
  SdoWrite(number, can_id, WRITE_1_BYTE, 0x1600, 0x00, 0x02);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 使能RPDO1
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x1400, 0x01, 0x00000200 + id);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
//  // 写入EEPROM
//  SdoWrite(number, can_id, WRITE_2_BYTES, 0x2010, 0x00, 1);
//  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
//  &read_data);
#endif
}
#if DRIVER_TYPE == TT_DRIVER
void SetSpeed(const uint8_t number, const uint32_t can_id,
              const int32_t value) {
  uint8_t buffer[4] = {0};
  *((int32_t *)&buffer[0]) = value;
  WriteData(number, can_id, buffer, 4);
}
#elif DRIVER_TYPE == ZL_DRIVER
void SetSpeed(const uint8_t number, const uint32_t can_id,
              const int32_t l_value, const int32_t r_value) {
  uint8_t buffer[8] = {0};
  *((int32_t *)&buffer[0]) = l_value;
  *((int32_t *)&buffer[4]) = r_value;
  WriteData(number, can_id, buffer, 8);
}

void SetRelativePosition(const uint8_t number, const uint32_t can_id,
                         const int32_t l_value, const int32_t r_value) {
  uint32_t read_can_id;
  uint8_t read_code;
  uint16_t read_index;
  uint8_t read_sub_index;
  uint32_t read_data;
  // 左电机 目标位置
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x607A, 0x01, l_value);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 右电机 目标位置
  SdoWrite(number, can_id, WRITE_4_BYTES, 0x607A, 0x02, r_value);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  // 启动相对运动
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x4F);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
  SdoWrite(number, can_id, WRITE_2_BYTES, 0x6040, 0x00, 0x5F);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          &read_data);
}

int GetStatus(const uint8_t number, const uint32_t can_id, uint16_t *l_value,
              uint16_t *r_value) {
  uint8_t buffer[8] = {0x40, 0x41, 0X60, 0x00, 0, 0, 0, 0};
  uint32_t read_can_id;
  uint8_t read_code;
  uint16_t read_index;
  uint8_t read_sub_index;
  uint32_t value;
  WriteData(number, can_id, buffer, 8);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          (uint32_t *)&value);
  if (read_code == 0x43) {
    *l_value = value & 0x0000FFFF;
    *r_value = value >> 16;
    return 0;
  }
  return -1;
}

#endif

void ResetNode(const uint8_t number, const uint8_t node_id) {
  uint8_t buffer[2] = {0x81, 0x01};
  // 复位节点
  buffer[1] = node_id;
  WriteData(number, 0, buffer, 2);
  // 初始化节点
  buffer[0] = 0x80;
  WriteData(number, 0, buffer, 2);

  CanDelayMs(50);
  uint32_t can_id;
  uint8_t size;
  uint8_t value[8];
  ReadData(number, &can_id, value, &size);
}

void StartNode(const uint8_t number, const uint8_t node_id) {
  uint8_t buffer[2] = {0x01, 0x01};
  buffer[1] = node_id;
  WriteData(number, 0, buffer, 2);
}

void GetRealtimeSpeed(const uint8_t number, int32_t *l_value,
                      int32_t *r_value) {
  int32_t buffer[2];
  uint32_t can_id;
  uint8_t size;
  ReadData(number, &can_id, (uint8_t *)buffer, &size);
  *l_value = buffer[0];
  *r_value = buffer[1];
}

void GetSpeed(const uint8_t number, const uint32_t can_id, int32_t *l_value,
              int32_t *r_value) {
  uint8_t buffer[8] = {0x40, 0x6C, 0X60, 0x01, 0, 0, 0, 0};
  uint32_t read_can_id;
  uint8_t read_code;
  uint16_t read_index;
  uint8_t read_sub_index;
  int32_t value;
  WriteData(number, can_id, buffer, 8);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          (uint32_t *)&value);
  //  printf("id = %X", read_can_id);
  if (read_sub_index == 1) {
    *l_value = value;
  } else if (read_sub_index == 2) {
    *r_value = value;
  }
  buffer[3] = 2;
  WriteData(number, can_id, buffer, 8);
  //  CanDelayMs(500);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          (uint32_t *)&value);
  //  printf("id = %X", read_can_id);
  if (read_sub_index == 1) {
    *l_value = value;
  } else if (read_sub_index == 2) {
    *r_value = value;
  }
}

void GetRealPosition(const uint8_t number, const uint32_t can_id,
                     int32_t *l_value, int32_t *r_value) {
  uint8_t buffer[8] = {0x40, 0x64, 0X60, 0x01, 0, 0, 0, 0};
  uint32_t read_can_id;
  uint8_t read_code;
  uint16_t read_index;
  uint8_t read_sub_index;
  int32_t value;
  WriteData(number, can_id, buffer, 8);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          (uint32_t *)&value);
  //  printf("id = %X", read_can_id);
  if (read_sub_index == 1) {
    *l_value = value;
  } else if (read_sub_index == 2) {
    *r_value = value;
  }
  buffer[3] = 2;
  WriteData(number, can_id, buffer, 8);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          (uint32_t *)&value);
  //  printf("id = %X", read_can_id);
  if (read_sub_index == 1) {
    *l_value = value;
  } else if (read_sub_index == 2) {
    *r_value = value;
  }
}

void ClearError(const uint8_t number, const uint32_t can_id) {
  uint8_t buffer[8] = {0x2B, 0x40, 0X60, 0x00, 0x80, 0, 0, 0};
  uint32_t read_can_id;
  uint8_t read_code;
  uint16_t read_index;
  uint8_t read_sub_index;
  uint32_t value;
  WriteData(number, can_id, buffer, 8);
  SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
          (uint32_t *)&value);
}
