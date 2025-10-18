#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <geometry_msgs/Twist.h>

#include "namiashi_control_hw/namiashi_hardware_interface.h"

enum class State {
  kBooting   = 0,
  kWaitStart = 1,
  kStarting  = 2,
  kRunning   = 3
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "namiashi_control_hw_node");
    ros::NodeHandle nh;

    dp::NamiashiHardwareInterface robot_hw;
    controller_manager::ControllerManager cm(&robot_hw, nh);

    std::string port;
    int baudrate;

    State state = State::kBooting;
    ROS_INFO("Trans to Booting : %d", state);

    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 10,
        [&](const geometry_msgs::Twist::ConstPtr& msg) {
            // ROS_INFO("cmdvel received");
            if (msg->linear.z >= 0.9) {
                // ROS_INFO("Go to default pose command received!");
                if (state == State::kWaitStart) {
                    ROS_INFO("Start moving to default pose! : %d", state);
                    state = State::kStarting;
                    robot_hw.default_pose();
                    ROS_INFO("Finish moving to default pose! : %d", state);
                    state = State::kRunning;
                }
            }
        }
    );

    nh.param<std::string>("control/serial_port", port, "/dev/ttyUSB0");
    nh.param<int>("control/baudrate", baudrate, 1000000);

    ROS_ERROR("SERIAL DEVICE : %s baud:%d", port.c_str(), baudrate);

    robot_hw.init(port, baudrate);
    state = State::kWaitStart;
    ROS_INFO("Trans to WaitStart : %d", state);
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate rate(100); // 50Hz制御ループ
    while (ros::ok())
    {
        // ROS_INFO("Waiting Trigger (cmd_vel) : %d", state);
        if (state == State::kRunning) {
            ROS_INFO("Received Trigger (cmd_vel) : %d", state);
            break;
        }
        // ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        if (state == State::kRunning) {
            auto duration = ros::Duration(1.0 / 100.0);
            robot_hw.read();
            cm.update(ros::Time::now(), duration);
            robot_hw.write();
        }
        rate.sleep();
    }
    return 0;
}