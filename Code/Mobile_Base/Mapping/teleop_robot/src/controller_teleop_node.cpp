/*
* Code: Custom PS4 controller teleoperation node for Grabby
* Project: Grabby
* Author: Julius Ortstadt
* Date: 03.12.2025
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class TeleopNode {
public:
    TeleopNode() {
        // Node handle
        ros::NodeHandle nh;

        // Subscriber
        joy_sub_ = nh.subscribe("/joy", 1, &TeleopNode::joyCallback, this);

        // Publisher: queue_size = 1
        cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_serial", 1);

        // Publish at 10 Hz
        timer_ = nh.createTimer(ros::Duration(0.1), &TeleopNode::publishCmd, this);

        ROS_INFO("Teleop node started (10 Hz output)");
    }

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
        //geometry_msgs::Twist cmd;

        float steer = msg->axes[0];   // Left stick horizontal
        float L2 = msg->axes[2];      // Left trigger
        float R2 = msg->axes[5];      // Right trigger

        // Forward/backward logic
        if (R2 < 0 && std::abs(R2) > 0.9) // R2 pressed = -1.0 (move forward)
            cmd.linear.x = 1.0;
        else if (L2 < 0 && std::abs(L2) > 0.9) // L2 pressed = -1.0 (move backward)
            cmd.linear.x = -1.0;
        else
            cmd.linear.x = 0.0;

        // Differential turns
        if (std::abs(steer) < 0.2) // dead zone
            cmd.angular.z = 0.0;
        else if (steer > 0 && std::abs(steer) > 0.8) // left static turn
            cmd.angular.z = 1.0;
        else if (steer < 0 && std::abs(steer) > 0.8) // right static turn
            cmd.angular.z = -1.0;

        //cmd_pub_.publish(cmd);
    }

     void publishCmd(const ros::TimerEvent&) {
        cmd_pub_.publish(cmd);
    }

    ros::Subscriber joy_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer timer_;

    geometry_msgs::Twist cmd;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "teleop_robot");

    TeleopNode teleop;

    ros::spin();

    return 0;
}
