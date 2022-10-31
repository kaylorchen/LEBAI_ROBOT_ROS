//
// Created by kaylor on 10/25/22.
//

#ifndef MOTOR_DRIVER_CAN_H
#define MOTOR_DRIVER_CAN_H

#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include "linux/can/error.h"
#include <unistd.h>
#include "stdint.h"

//#define CAN_DEBUG

#ifdef __cplusplus
extern "C" {
#endif

int CanInit(int *can_fd, const char *device);

void CanFiltersConfig(int *can_fd, const void *rfilter, int size);

void CanWrite(int *can_fd, uint32_t can_id, const uint8_t *data, uint8_t size);

void CanRead(int *can_fd, uint32_t *can_id, uint8_t *data, uint8_t *size);

#ifdef __cplusplus
};
#endif

#endif //MOTOR_DRIVER_CAN_H
