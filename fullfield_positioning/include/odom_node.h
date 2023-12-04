#ifndef __ODOM_NODE_H
#define __ODOM_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "bupt_can/bupt_can.h"
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <time.h>


// 全长定位数据
typedef struct
{
    float vx;//x轴速度(m/s)
    float x;//x轴坐标(m)
    float vy;//y轴速度(m/x)
    float y;//y轴坐标(m)
    float vyaw;//偏航角速度(dps)
    float yaw;//偏航角(degree)
} fullFieldPositioning_data;

// 四元数
typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

class OdometerPublisher : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    OdometerPublisher(std::string name);
    //获取x轴数据
    void getXdata(const std::shared_ptr<can_frame> &frame);
    //获取y轴数据
    void getYdata(const std::shared_ptr<can_frame> &frame);
    //获取yaw数据
    void getYawData(const std::shared_ptr<can_frame> &frame);
    //发布里程计消息
    void publish_msg();
private:
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_node;
    // 发布的里程计消息
    nav_msgs::msg::Odometry _odom;
    //从全长定位获取的速度
    fullFieldPositioning_data _fullFieldPositioning_data;
    //四元数
    quaternion_t _quaternion;

    //欧拉角生成四元数
    void Euler2Quaternion(float yaw, float roll=0, float pitch=0);
    //根据受到数据生成里程计消息
    void CreateOdomMsg();
};

#endif //__ODOM_NODE_H
