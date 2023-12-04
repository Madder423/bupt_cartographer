#include "odom_node.h"

const float Pi  = 3.1415926535;


float angle2pi(float x)
{
      x = x / 180 * Pi  ;
    if (x > Pi)             
    {                      
        x -= 2 * Pi;
    }                       
    else if (x < -Pi)   
    {                       
        x += 2 * Pi;
    }
    return x;
}



OdometerPublisher::OdometerPublisher(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 创建发布者
        odom_node = this->create_publisher<nav_msgs::msg::Odometry>("odom",10);
        // 创建定时器，15ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(15), std::bind(&OdometerPublisher::publish_msg, this));

    }

void OdometerPublisher::CreateOdomMsg()
{
    //消息头
    _odom.header.stamp.sec = this->now().seconds();
    _odom.header.stamp.nanosec = (this->now().nanoseconds()%1000000000);
    std::cout<<_odom.header.stamp.sec<<std::endl;
    std::cout<<_odom.header.stamp.nanosec<<std::endl;
                                                                        
    _odom.header.frame_id = "odom";
    _odom.child_frame_id = "odom";
    //姿态
    Euler2Quaternion(_fullFieldPositioning_data.yaw);//计算四元数
    _odom.pose.pose.position.x = _fullFieldPositioning_data.x;
    _odom.pose.pose.position.y = _fullFieldPositioning_data.y;
    _odom.pose.pose.orientation.w = _quaternion.w;
    _odom.pose.pose.orientation.y = _quaternion.y;
    _odom.pose.pose.orientation.x = _quaternion.x;
    _odom.pose.pose.orientation.z = _quaternion.z;
    //速度
    _odom.twist.twist.angular.z = _fullFieldPositioning_data.vyaw;
    _odom.twist.twist.linear.x = _fullFieldPositioning_data.vx;
    _odom.twist.twist.linear.y = _fullFieldPositioning_data.vy;
}

void OdometerPublisher::getXdata(const std::shared_ptr<can_frame> &frame)
{
    //RCLCPP_INFO(this->get_logger(),"getXdata");
    _fullFieldPositioning_data.vx=(*(float*)frame->data);
    _fullFieldPositioning_data.x=(*(float*)(frame->data+4));
    //     //测试代码
    // std::cout<<_fullFieldPositioning_data.vx<<std::endl;
    // std::cout<<_fullFieldPositioning_data.x<<std::endl;
}


void OdometerPublisher::getYdata(const std::shared_ptr<can_frame> &frame)
{
    //RCLCPP_INFO(this->get_logger(),"getYdata");
    //std::cout<<(*(float*)frame->data)<<" "<<(*(float*)(frame->data+4))<<"\n";
    _fullFieldPositioning_data.vy=(*(float*)frame->data);
    _fullFieldPositioning_data.y=(*(float*)(frame->data+4));
    //     //测试代码
    // std::cout<<_fullFieldPositioning_data.vy<<std::endl;
    // std::cout<<_fullFieldPositioning_data.y<<std::endl;
}
    


void OdometerPublisher::getYawData(const std::shared_ptr<can_frame> &frame)
{
    //RCLCPP_INFO(this->get_logger(),"getYawData");
    _fullFieldPositioning_data.vyaw=angle2pi(*(float*)frame->data);
    _fullFieldPositioning_data.yaw=angle2pi(*(float*)(frame->data+4));
    // //测试代码
    // std::cout<<_fullFieldPositioning_data.vyaw<<std::endl;
    // std::cout<<_fullFieldPositioning_data.yaw<<std::endl;
}

void OdometerPublisher::Euler2Quaternion(float yaw, float roll, float Pitch)

{
    // 传入机器人的欧拉角 roll、Pitch 和 yaw。
    // 计算欧拉角的 sin 和 cos 值，分别保存在 cr、sr、cy、sy、cp、sp 六个变量中   
    // https://blog.csdn.net/xiaoma_bk/article/details/79082629
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(Pitch * 0.5);
    double sp = sin(Pitch * 0.5);
    // 计算出四元数的四个分量 q.w、q.x、q.y、q.z
    _quaternion.w = cy * cp * cr + sy * sp * sr;
    _quaternion.x = cy * cp * sr - sy * sp * cr;
    _quaternion.y = sy * cp * sr + cy * sp * cr;
    _quaternion.z = sy * cp * cr - cy * sp * sr;
    std::cout<<_quaternion.w<<std::endl;
}

void OdometerPublisher::publish_msg()
{
    //测试代码
    //std::cout<<"hello world"<<std::endl;
    //根据受到数据生成里程计消息
    CreateOdomMsg();
    // 日志打印
    RCLCPP_INFO(this->get_logger(),"\t\nx=%f\t\nvx=%f\t\ny=%f\t\nvy=%f\t\nyaw=%f\t\nvyaw:%f\t\n",_fullFieldPositioning_data.x,_fullFieldPositioning_data.vx,_fullFieldPositioning_data.y,_fullFieldPositioning_data.vy,_fullFieldPositioning_data.yaw,_fullFieldPositioning_data.vyaw);
    odom_node->publish(_odom);
}

