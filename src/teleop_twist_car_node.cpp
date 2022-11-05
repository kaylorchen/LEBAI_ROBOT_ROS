#include "driver/driver_can.h"
#include "driver/tt_driver.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sys/epoll.h"

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
  struct can_filter rfilter[3];
  rfilter[0].can_id = SDO_TX_ID(1);
  rfilter[0].can_mask = CAN_SFF_MASK;  // 标准帧 (SFF: standard frame format)

  rfilter[1].can_id = HEARTBEAT_ID(1);
  rfilter[1].can_mask = CAN_SFF_MASK;

  rfilter[2].can_id = TPDO1_ID(1);
  rfilter[2].can_mask = CAN_SFF_MASK;
  CanFiltersConfig(&can0, &rfilter, sizeof(rfilter));

  // 复位节点1
  ResetNode(0, 1);
  // 设置节点1为速度模式
  EnableSpeedMode(0, SDO_RX_ID(1));
  // 配置节点1的RPDO1
  EnableRpdo1(0, SDO_RX_ID(1));
  // 配置节点1的TPDO1
  EnableTpdo1(0, SDO_RX_ID(1));
  // 开启节点，开启PDO模式
  StartNode(0, 1);
}

double kWheelDistance = 229.8 / 1000;
double kLeftWheelRadius = 60.2 / 1000;
double kRightWheelRadius = 60.2 / 1000;
#include "math.h"

int32_t double2int(double value) {
  int32_t ret;
  if (value > 0) {
    ret = (value * 10 + 5) / 10;
  } else if (value < 0) {
    ret = (value * 10 - 5) / 10;
  } else {
    ret = 0;
  }
  return ret;
}

void Callback(const geometry_msgs::Twist::ConstPtr &msg) {
  double linear = msg->linear.x;
  double angular = msg->angular.z;
  double v_l = linear - kWheelDistance * angular / 2;
  double v_r = linear + kWheelDistance * angular / 2;
  int32_t rpm_l = double2int(30.0 * v_l / M_PI / kLeftWheelRadius);
  int32_t rpm_r = double2int(30.0 * v_r / M_PI / kRightWheelRadius);
  ROS_INFO("linear: %lf m/s, angular: %lf rad/s, left = %d rpm, right = %d rpm",
           linear, angular, rpm_l, rpm_r);
  // 因为安装的问题，所以左轮速度应该取负再发送
  SetSpeed(0, RPDO1_ID(1), -rpm_l, rpm_r);
}

#define MAXEVENTS (64)

int main(int argc, char **argv) {
  InitDevice();
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  double kRightCorrectionFactor;
  double kLeftCorrectionFactor;
  double kWheelbase;
  double kAverageWheelRadius;
  n.param<double>("/kRightCorrectionFactor", kRightCorrectionFactor, 1.0);
  n.param<double>("/kLeftCorrectionFactor", kLeftCorrectionFactor, 1.0);
  n.param<double>("/kWheelbase", kWheelbase, 0.2298);
  n.param<double>("/kAverageWheelRadius", kAverageWheelRadius, 0.0602);
  kWheelDistance = kWheelbase;
  kLeftWheelRadius = kAverageWheelRadius * kLeftCorrectionFactor;
  kRightWheelRadius = kAverageWheelRadius * kRightCorrectionFactor;

  ROS_INFO(
      "\nkWheelbase = %lf\n kLeftWheelRadius = %lf\n kRightWheelRadius = %lf",
      kWheelbase, kLeftWheelRadius, kRightWheelRadius);
  ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, Callback);

  int efd;
  struct epoll_event event;
  struct epoll_event *events;
  efd = epoll_create(MAXEVENTS);
  if (efd == -1) {
    perror("epoll_create");
    abort();
  }
  event.data.fd = can0;
  event.events = EPOLLIN;
  ROS_INFO("can fd = %d", can0);
  int s = epoll_ctl(efd, EPOLL_CTL_ADD, can0, &event);
  if (s == -1) {
    perror("epoll_ctl");
    abort();
  }

  /* Buffer where events are returned */
  events = (struct epoll_event *)calloc(MAXEVENTS, sizeof event);
  while (ros::ok()) {
    int num = epoll_wait(efd, events, MAXEVENTS, 2);

    for (size_t i = 0; i < num; i++) {
      if ((events[i].events & EPOLLERR) || (events[i].events & EPOLLHUP) ||
          (!(events[i].events & EPOLLIN))) {
        /* An error has occured on this fd, or the socket is not
           ready for reading (why were we notified then?) */
        fprintf(stderr, "epoll error\n");
        close(events[i].data.fd);
        continue;
      } else if (events[i].events & EPOLLIN) {
        int32_t _l_value, _r_value;
        GetRealtimeSpeed(0, &_l_value, &_r_value);
        // 安装原因，左轮数值取反
        _l_value = -_l_value;
        double l_velocity = (_l_value * M_PI * kLeftWheelRadius) / 300.0;
        double r_velocity = (_r_value * M_PI * kRightWheelRadius) / 300.0;
        double velocity = (l_velocity + r_velocity) / 2;
        double angular_velocity = (r_velocity - l_velocity) / kWheelDistance;
        static double x_world = 0.0, y_world = 0.0, theta = 0.0;
        x_world += velocity * SAMPLING_PERIOD / 1000.0 * cos(theta);
        y_world += velocity * SAMPLING_PERIOD / 1000.0 * sin(theta);
        theta += angular_velocity * SAMPLING_PERIOD / 2000.0;
        // ROS_INFO("x = %lf, y = %lf, theta = %lf, v = %lf, w = %lf", x_world,
        // y_world, theta/M_PI*180, velocity, angular_velocity); ROS_INFO("num =
        // %d, l = %d, r = %d", num, _l_value, _r_value);
      }
    }
    ros::spinOnce();
  }
  free(events);
  return 0;
}
