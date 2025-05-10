#include <ros/ros.h>
#include "crsf_receiver.h"

int main(int argc, char** argv)
{
    // ROS1 初始化
    ros::init(argc, argv, "crsf_receiver");  // 节点名称作为参数
    
    // 创建节点句柄
    ros::NodeHandle nh;
    
    // 创建节点实例（假设 CrsfReceiverNode 已改为 ROS1 风格）
    CrsfReceiverNode node(nh);  // 传入节点句柄
    
    // ROS1 主循环
    ros::spin();
    
    return 0;
}