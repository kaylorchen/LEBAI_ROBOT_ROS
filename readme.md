# 配置Jetson的CAN0接口
## 开启加载mttcan驱动
Jetson开发版image把can的驱动默认使用黑名单模式屏蔽了，我们现在需要开机默认加载驱动
```bash
cd /etc/modprobe.d/
sudo mv blacklist-mttcan.conf blacklist-mttcan.conf.bak
```

## 配置CAN0的参数
编辑 ***/etc/network/interface***, 添加以下内容
```bash
auto can0
iface can0 inet manual
pre-up busybox devmem 0x0c303000 32 0x0000C400
pre-up busybox devmem 0x0c303008 32 0x0000C458
pre-up ip link set can0 type can bitrate 500000
pre-up ip link set can0 type can restart-ms 1
```
以上配置首先配置了Jetson的can接口的PIN，然后配置can接口的通信速率为500k，失效重启时间为1ms。

如果没有busybox，请使用 ***sudo apt install busybox***

# 安装手柄相关ROS包
```bash
sudo apt install ros-noetic-joy ros-noetic-joy-teleop
```
根据自己使用的ros版本，替换上面的noetic

# 编译运行本程序
## 安装 （略）
## 启动
```bash
roslaunch teleop_twist_car teleop_twist_car.launch
```