/*
* Code: Custom keyboard teleoperation node for Grabby allowing us to control it from the keyboard through ROS
* Project: Grabby Warehouse Robot
* Author: Julius Ortstadt
* Date: 09.12.2024
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <fcntl.h>


class TeleopNode{
    public:
        TeleopNode(){
            // Initialize the ROS node handle and publisher
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("teleop_twist", 10);
            linear_speed_ = 1.0;
            angular_speed_ = 1.0;

            std::cout << "Teleop node initialized.\n";
            std::cout << "Keybindings are as follows:\n";
            std::cout << " W: Move forward\n";
            std::cout << " S: Move backward\n";
            std::cout << " A: Move left\n";
            std::cout << " D: Move right\n";
            std::cout << " Q: Exit the controller\n";
        }

        void run(){
            char key;
            ros::Rate loop_rate(10); // Set loop rate

            while (ros::ok()){
                key = getKey();

                // Reset velocities
                geometry_msgs::Twist twist;
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;

                // Map keys to actions 
                if (key == 'w' || key == 'W') twist.linear.x = linear_speed_;
                else if (key == 's' || key == 'S') twist.linear.x = -linear_speed_;
                else if (key == 'a' || key == 'A') twist.angular.z = angular_speed_;
                else if (key == 'd' || key == 'D') twist.angular.z = -angular_speed_;
                else if (key == 'q' || key == 'Q') {
                    std::cout << "Exiting keyboard controller...\n"; 
                    break;
                }
                else std::cout << "Invalid key. Use WASD keys.\n";

                // Publish the twist message
                twist_pub_.publish(twist);

                // Sleep to maintain loop rate
                loop_rate.sleep();
            }
        }        

    private:
        ros::NodeHandle nh_;
        ros::Publisher twist_pub_;
        double linear_speed_;
        double angular_speed_;

        // Function to get a single character from keyboard input
        char getKey(){
            struct termios oldt, newt;
            char ch;
            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            ch = getchar();
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            return ch;
        }
}; 



int main(int argc, char* argv[]){
    // Initialize ROS 
    ros::init(argc, argv, "teleop_node");
    
    // Create an instance of the node and run it
    TeleopNode controller;
    controller.run();

    // Mandatory if node subscribes to something
    //ros::spin();

    return 0;
}