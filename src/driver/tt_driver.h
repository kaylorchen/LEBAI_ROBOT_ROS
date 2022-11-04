//
// Created by kaylor on 10/25/22.
//

#ifndef MOTOR_DRIVER_TT_DRIVER_H
#define MOTOR_DRIVER_TT_DRIVER_H

#include "stdint.h"
#include "stdio.h"

#define TT_DRIVER 0
#define ZL_DRIVER 1
#define DRIVER_TYPE ZL_DRIVER
#define SDO_DELAY (1)

#if DRIVER_TYPE == TT_DRIVER
#elif DRIVER_TYPE == ZL_DRIVER
#define ENCODER_TOTAL_COUNT (0x4000)
#endif

/**
 * 主站写给从站的命令符功能码
 * */
#define WRITE_4_BYTES (0x23)
#define WRITE_2_BYTES (0x2B)
#define WRITE_1_BYTE (0x2F)
#define READ_DATA (0x40)

/**
 * 从站回应主站的命令符功能码
 * */
#define READ_FEEDBACK_4_BYTES (0x43)
#define READ_FEEDBACK_2_BYTES (0x4B)
#define READ_FEEDBACK_1_BYTE (0x4F)
#define WRITE_FEEDBACK (0x60)
#define ERROR_FEEDBACK (0x80)

/**
 * 这里的RX和TX是相对与驱动来说的，即：
 * RX表示主站发往从站
 * TX表示从站发往主站
 * */
#define HEARTBEAT_ID(id) (id + 0x700)
#define SDO_RX_ID(id) (id + 0x600)
#define SDO_TX_ID(id) (id + 0x580)
#define TPDO1_ID(id) (id + 0x180)
#define TPDO2_ID(id) (id + 0x280)
#define TPDO3_ID(id) (id + 0x380)
#define TPDO4_ID(id) (id + 0x480)
#define RPDO1_ID(id) (id + 0x200)
#define RPDO2_ID(id) (id + 0x300)
#define RPDO3_ID(id) (id + 0x400)
#define RPDO4_ID(id) (id + 0x500)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * CanDelayMs(), WriteData()和ReadData()需要根据实际情况重写实现函数
 * */

void CanDelayMs(const uint32_t count);

void WriteData(const uint8_t number, const uint32_t can_id, const uint8_t *data,
               const uint8_t size);

void ReadData(const uint8_t number, uint32_t *can_id, uint8_t *data,
              uint8_t *size);

void ResetNode(const uint8_t number, const uint8_t node_id);

void StartNode(const uint8_t number, const uint8_t node_id);

void SdoWrite(const uint8_t number, const uint32_t can_id, const uint8_t code,
              const uint16_t index, const uint8_t sub_index,
              const uint32_t data);

void SdoRead(const uint8_t number, uint32_t *can_id, uint8_t *code,
             uint16_t *index, uint8_t *sub_index, uint32_t *data);

void EnableSpeedMode(const uint8_t number, const uint32_t can_id);

void EnablePositionMode(const uint8_t number, const uint32_t can_id);

/**
 * 使用RPDO1映射速度设置，实现实时控制
 * */
void EnableRpdo1(const uint8_t number, const uint32_t can_id);

void EnableTpdo1(const uint8_t number, const uint32_t can_id);

#if DRIVER_TYPE == TT_DRIVER
/**
 * 天太单位是 0.1rpm
 * 调用 SetSpeed(1, RPDO1_ID(2), 1000);
 * */
void SetSpeed(const uint8_t number, const uint32_t can_id, const int32_t value);
#elif DRIVER_TYPE == ZL_DRIVER
/**
 * 中菱单位是1rpm
 * 调用 SetSpeed(0, RPDO1_ID(2), 1000, -1000);
 * */
void SetSpeed(const uint8_t number, const uint32_t can_id,
              const int32_t l_value, const int32_t r_value);

/**
 * @brief Set the Relative Position object
 *
 * @param number can设备标号
 * @param can_id can ID
 * @param l_value 左电机的脉冲数
 * @param r_value 右电机的脉冲数
 */
void SetRelativePosition(const uint8_t number, const uint32_t can_id,
                         const int32_t l_value, const int32_t r_value);

/**
 * @brief Get the Status object
 *
 * @param number can设备标号
 * @param can_id can ID
 * @param l_value 左电机的状态字的指针
 * @param r_value 右电机的状态字的指针
 * @return int 0： 成功 -1： 失败
 */
int GetStatus(const uint8_t number, const uint32_t can_id, uint16_t *l_value,
              uint16_t *r_value);

void ClearError(const uint8_t number, const uint32_t can_id);

#endif

/**
 * 通过TPDO1得到实时速度
*/
void GetRealtimeSpeed(const uint8_t number, int32_t *l_value, int32_t *r_value);

/**
 * 获取左右电机速度， 中菱的单位为 0.1rpm
 * */
void GetSpeed(const uint8_t number, const uint32_t can_id, int32_t *l_value,
              int32_t *r_value);

/**
 * @brief Get the Real Position object
 *
 * @param number can设备标号
 * @param can_id can ID
 * @param l_value 左电机的脉冲数的指针
 * @param r_value 右电机的脉冲数的指针
 */
void GetRealPosition(const uint8_t number, const uint32_t can_id,
                     int32_t *l_value, int32_t *r_value);

#ifdef __cplusplus
};
#endif

#endif  // MOTOR_DRIVER_TT_DRIVER_H
