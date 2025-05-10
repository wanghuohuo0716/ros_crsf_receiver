#ifndef CRSF_RECEIVER_HPP
#define CRSF_RECEIVER_HPP

#include <chrono>
#include <functional>
#include <vector>

#include <std_msgs/String.h>	
#include <std_msgs/Int32MultiArray.h>	

#include <ros/ros.h>
//#include "rclcpp/qos.hpp"
#include <CppLinuxSerial/SerialPort.hpp>

#include "crsf_parser.h"
#include "crsf_receiver_msg/CRSFChannels16.h"
#include "crsf_receiver_msg/CRSFLinkInfo.h"

using namespace std::chrono_literals;
using namespace mn;
using namespace crsf_receiver_msg;


class CrsfReceiverNode
{
public:
  explicit CrsfReceiverNode(ros::NodeHandle& nh);  // 正确声明
  ~CrsfReceiverNode();

private:
  CppLinuxSerial::SerialPort serial;
  CrsfParser parser;

  std::string device;
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Publisher channels_pub_;
  ros::Publisher link_pub_;

    std::string device_;
    int baudrate_;
    bool link_stats_;
    int receiver_rate_;
    
  void mainTimerCallback(const ros::TimerEvent&);  // ← 这也要声明！
};


#endif 