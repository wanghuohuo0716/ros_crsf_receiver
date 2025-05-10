#include "crsf_receiver.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

CrsfReceiverNode::CrsfReceiverNode(ros::NodeHandle& )
{
    nh_.param<std::string>("device", device_, "/dev/ttyUSB0");
    nh_.param<int>("baudrate", baudrate_, CRSF_BAUDRATE);
    nh_.param<bool>("link_stats", link_stats_, true);
    nh_.param<int>("receiver_rate", receiver_rate_, 100);

    channels_pub_ = nh_.advertise<crsf_receiver_msg::CRSFChannels16>("rc/channels", 1);
    link_pub_ = nh_.advertise<crsf_receiver_msg::CRSFLinkInfo>("rc/link", 1);
    
    nh_.param<std::string>("device", device_, "/dev/ttyUSB0");
    nh_.param<int>("baudrate", baudrate_, 115200);  // 替代 CRSF_BAUDRATE 宏
    nh_.param<int>("receiver_rate", receiver_rate_, 100);
    int period = 1000 / receiver_rate_;

    ROS_INFO("Receiver receiver_rate is %dhz (period %dms)", receiver_rate_, period);
    ROS_INFO("Target serial device is: %s", device_.c_str());
    ROS_INFO("Selected baudrate: %d", baudrate_);

    timer_ = nh_.createTimer(
        ros::Duration(period / 1000.0),  // period 单位需转为秒
        &CrsfReceiverNode::mainTimerCallback, 
        this
    );

    serial.SetDevice(device_);
    serial.SetBaudRate(baudrate_);
    serial.SetTimeout(period / 2);
}

void CrsfReceiverNode::mainTimerCallback(const ros::TimerEvent&)
{
    if (serial.GetState() == CppLinuxSerial::State::CLOSED) {
        try {
            serial.Open();
        } catch (const CppLinuxSerial::Exception& e) {
            ROS_WARN("Cannot open serial port: %s", device_.c_str());
            return;
        }
    }

    if (serial.Available()) {
        serial.ReadBinary(parser.rx_buffer);
        parser.parse_incoming_bytes();
        
    }

    if (parser.is_channels_actual()) {

        CRSFChannels16 message = convert_to_channels_message(parser.get_channels_values());
        channels_pub_.publish(message);
    }

    if (parser.is_link_statistics_actual()) {

        CRSFLinkInfo message = convert_to_link_info(parser.get_link_info());
        link_pub_.publish(message);
    }
}

CrsfReceiverNode::~CrsfReceiverNode() {
    if(serial.GetState() == CppLinuxSerial::State::OPEN) {
        serial.Close();
    }
}
