//
// Created by kaylor on 10/25/22.
//
#include "stdio.h"
#include <string.h>
#include <stdint-gcc.h>
#include <stdlib.h>
#include "driver_can.h"

int CanInit(int *can_fd, const char *device) {
  *can_fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (*can_fd < 0) {
    perror("socket can create error!\n");
    return -1;
  }
  struct ifreq ifr;
  strcpy(ifr.ifr_name, device);
  ioctl(*can_fd, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  // 将套接字与 can0 绑定
  int bind_res = bind(*can_fd, (struct sockaddr *) &addr, sizeof(addr));
  if (bind_res < 0) {
    perror("bind error!");
    return -1;
  }
  return *can_fd;
}

void CanFiltersConfig(int *can_fd, const void *rfilter, int size) {
  setsockopt(*can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, size);
}

void CanWrite(int *can_fd, uint32_t can_id, const uint8_t *data, uint8_t size) {
  struct can_frame frame;
  memcpy(frame.data, data, size);
  frame.can_dlc = size;
  frame.can_id = can_id;
 int nbytes = write(*can_fd, &frame, sizeof(struct can_frame));
  if (nbytes != sizeof(frame)){
    fprintf(stderr, "write error, nbytes = %d\n", nbytes);
    exit(-1);
  }
#ifdef CAN_DEBUG
  else{
    printf("write successful, id = %X [%d] ", can_id, size);
    for (int i = 0; i < size; ++i) {
      printf("%02X ", *(data + i));
    }
    printf("\n");
  }
#endif
}

void CanRead(int *can_fd, uint32_t *can_id, uint8_t *data, uint8_t *size) {
  struct can_frame can_frame;
  int nbytes = read(*can_fd, &can_frame, sizeof(struct can_frame));
  if (nbytes < 0) {
    perror("can raw socket read");
    close(*can_fd);
    exit(-1);
  }

  if (nbytes < sizeof(struct can_frame)) {
    fprintf(stderr, "read: incomplete CAN frame\n");
    close(*can_fd);
    exit(-1);
  }
  *can_id = can_frame.can_id;
  *size = can_frame.can_dlc;
  memcpy(data, can_frame.data, can_frame.can_dlc);
#ifdef CAN_DEBUG
  printf("read successful id = %X [%d] ", *can_id, *size);
  for (int i = 0; i < *size; ++i) {
    printf(" %02X", *(data + i));
  }
  printf("\n");
#endif
}
