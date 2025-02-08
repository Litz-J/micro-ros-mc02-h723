# MicroROS2适配达妙MC-02开发板（STM32 H723）软件包
本项目是基于MicroROS2，利用CubeMX工具和micro_ros_stm32cubemx_utils软件包整合的一个MicroROS基本功能框架。

## 软件版本
CubeMX:6.11.0

## 使用
项目使用makefile脚本构建。需要使用Arm GNU Toolchain编译。安装后在makefile同级目录下执行：
```bash
make
```
或者
```bash
make -jxx   #多线程编译。xx为线程数
```
即可编译。

## 与ROS2（上位机端）通信
上位机端需要运行一个节点micro-ros-Agent，用于中转收发MicroROS2的通信数据包。

链接：https://github.com/micro-ROS/micro-ROS-Agent


## 本项目使用的工具包

[micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
