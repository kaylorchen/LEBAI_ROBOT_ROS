#include "driver/driver_can.h"
#include "driver/tt_driver.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

int can0;
void CanDelayMs(const uint32_t count) { usleep(1000 * count); }

void WriteData(const uint8_t number, const uint32_t can_id, const uint8_t *data,
               const uint8_t size) {
  int fd = -1;
  switch (number) {
    case 0:
      fd = can0;
      break;
    default:
      break;
  }
  CanWrite(&fd, can_id, data, size);
}

void ReadData(const uint8_t number, uint32_t *can_id, uint8_t *data,
              uint8_t *size) {
  int fd = -1;
  switch (number) {
    case 0:
      fd = can0;
      break;
    default:
      break;
  }
  CanRead(&fd, can_id, data, size);
}

void InitDevice(void) {
  CanInit(&can0, "can0");
  /********************* 过滤规则设置 *********************/
  /**
   * 中菱电机默认是ID是1，
   * 我们这里过滤器使用过滤0x581（SDO反馈包）和0x701（心跳包）
   * **/
  struct can_filter rfilter[2];
  rfilter[0].can_id = SDO_TX_ID(1);
  rfilter[0].can_mask = CAN_SFF_MASK;  // 标准帧 (SFF: standard frame format)

  rfilter[1].can_id = HEARTBEAT_ID(1);
  rfilter[1].can_mask = CAN_SFF_MASK;
  CanFiltersConfig(&can0, &rfilter, sizeof(rfilter));

  // 复位节点1
  ResetNode(0, 1);
  // 设置节点1为速度模式
  EnableSpeedMode(0, SDO_RX_ID(1));
  // 配置节点1的RPDO1
  EnableRpdo1(0, SDO_RX_ID(1));
  // 开启节点，开启PDO模式
  StartNode(0, 1);
}

const double kWheelDistance = 229.8 / 1000 * 0.990666;
const double kWheelRadius = 60.2 / 1000;
#include "math.h"
void Callback(const geometry_msgs::Twist::ConstPtr &msg) {
  double linear = msg->linear.x;
  double angular = msg->angular.z;
  double v_l = linear - kWheelDistance * angular / 2;
  double v_r = linear + kWheelDistance * angular / 2;
  int32_t rpm_l = 30 * v_l / M_PI / kWheelRadius;
  int32_t rpm_r = 30 * v_r / M_PI / kWheelRadius;
  ROS_INFO("linear: %lf m/s, angular: %lf rad/s, left = %d rpm, right = %d rpm",
           linear, angular, rpm_l, rpm_r);
  // 因为安装的问题，所以左轮速度应该取负再发送
  SetSpeed(0, RPDO1_ID(1), -rpm_l, rpm_r);
}

const float kSquare_L = 1.0f;

int main(int argc, char **argv) {
  InitDevice();
  EnablePositionMode(0, SDO_RX_ID(1));
  int count = 1;
  while (count--) {
    uint16_t l_status;
    uint16_t r_status;
    uint32_t read_can_id;
    uint8_t read_code;
    uint16_t read_index;
    uint8_t read_sub_index;
    uint32_t read_data;
    uint8_t number = 0;
    uint16_t can_id = SDO_RX_ID(1);
    // 设置左电机 最大速度20rpm
    SdoWrite(number, can_id, WRITE_4_BYTES, 0x6081, 0x01, 30);
    SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
            &read_data);
    // 设置右电机 最大速度20rpm
    SdoWrite(number, can_id, WRITE_4_BYTES, 0x6081, 0x02, 30);
    SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
            &read_data);

    SetRelativePosition(
        0, SDO_RX_ID(1),
        -ENCODER_TOTAL_COUNT * 10,
        ENCODER_TOTAL_COUNT * 10);
    do {
      GetStatus(0, SDO_RX_ID(1), &l_status, &r_status);
      ROS_INFO("l = 0x%04X, r = 0x%04X", l_status & 0x8000, r_status & 0x8000);
    } while (((l_status & 0x8000) == 0) && ((l_status & 0x8000) == 0));

    // // 设置左电机 最大速度3rpm
    // SdoWrite(number, can_id, WRITE_4_BYTES, 0x6081, 0x01, 1);
    // SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
    //         &read_data);
    // // 设置右电机 最大速度3rpm
    // SdoWrite(number, can_id, WRITE_4_BYTES, 0x6081, 0x02, 1);
    // SdoRead(number, &read_can_id, &read_code, &read_index, &read_sub_index,
    //         &read_data);

    // SetRelativePosition(
    //     0, SDO_RX_ID(1),
    //     -ENCODER_TOTAL_COUNT * kWheelDistance / (8 * kWheelRadius * 0.98849),
    //     -ENCODER_TOTAL_COUNT * kWheelDistance / (8 * kWheelRadius* 1.01151));
    // do {
    //   GetStatus(0, SDO_RX_ID(1), &l_status, &r_status);
    //   ROS_INFO("l = 0x%04X, r = 0x%04X", l_status & 0x8000, r_status & 0x8000);
    // } while (((l_status & 0x8000) == 0) && ((l_status & 0x8000) == 0));
  }
  // ros::init(argc, argv, "listener");
  // ros::NodeHandle n;
  // ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, Callback);
  // ros::spin();
  ROS_INFO("Exit");
  return 0;
}
