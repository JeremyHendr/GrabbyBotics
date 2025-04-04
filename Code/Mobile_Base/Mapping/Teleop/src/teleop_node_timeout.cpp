/*
* Code: Custom keyboard teleoperation node for Grabby allowing us to control it from the keyboard through ROS
* Project: Grabby Warehouse Robot
* Author: Julius Ortstadt
* Date: 16.12.2024
*/

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <fcntl.h>
#include <cctype>


class TeleopNode{
    public:
        TeleopNode(){
            // Initialize the ROS node handle and publisher
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>("teleop_timeout", 10);
            linear_speed_ = 1.0;
            angular_speed_ = 1.0;
            timeout_duration_ = std::chrono::seconds(3); // Stop robot after 3 seconds of inactivity
            last_command_time_ = std::chrono::steady_clock::now();

            std::cout << "Teleop node initialized.\n";
            std::cout << "Keybindings are as follows:\n";
            std::cout << " W: Move forward\n";
            std::cout << " S: Move backward\n";
            
            std::cout << " A: Left turn\n";
            std::cout << " D: Right turn\n";
            
            std::cout << " Q: Forward left turn\n";
            std::cout << " E: Forward right turn\n";
            
            std::cout << " Y: Backward left turn\n";
            std::cout << " C: Backward right turn\n";
            
            std::cout << " O: Stop";
            std::cout << " X: Exit the controller\n";
        }

        void run(){
            char key;
            ros::Rate loop_rate(10); // Set loop rate

            while (ros::ok()){
                // Get the current time
                auto now = std::chrono::steady_clock::now();

                // Check for a timeout 
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_command_time_) >= timeout_duration_){
                    // If timeout occurs, stop the robot
                    stopRobot();
                    last_command_time_ = now;
                }

                // Check for new key input
                if (isKeyAvailable()){
                    key = getKey();
                    processKey(key);    
                }

                // Sleep to maintain loop rate
                loop_rate.sleep();
            }
        }        

    private:
        ros::NodeHandle nh_;
        ros::Publisher twist_pub_;
        double linear_speed_;
        double angular_speed_;
        std::chrono::seconds timeout_duration_;
        std::chrono::steady_clock::time_point last_command_time_;

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

        // Check if a key is available without blocking
        bool isKeyAvailable() {
            struct termios oldt, newt;
            int oldf;
            char ch;

            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

            ch = getchar();

            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
            fcntl(STDIN_FILENO, F_SETFL, oldf);

            if (ch != EOF) {
                ungetc(ch, stdin);
                return true;
            }

            return false;
        }

        // Process the input key and update the Twist message
        void processKey(char key) {
            geometry_msgs::Twist twist;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            bool al_key = false;

            // Map keys to actions 
            if (std::isalnum(key)){
                al_key = true;
                // Forward - Backward
                if (key == 'w' || key == 'W') twist.linear.x = linear_speed_;
                else if (key == 's' || key == 'S') twist.linear.x = -linear_speed_;
                // Static Left - Right
                else if (key == 'a' || key == 'A') twist.angular.z = -angular_speed_;
                else if (key == 'd' || key == 'D') twist.angular.z = angular_speed_;
                // Moving Forward - Left/Right
                else if (key == 'q' || key == 'Q'){
                    twist.linear.x = linear_speed_;
                    twist.angular.z = -angular_speed_;
                } 
                else if (key == 'e' || key == 'E'){
                    twist.linear.x = linear_speed_;
                    twist.angular.z = angular_speed_;
                } 
                // Moving Backward - Left/Right
                else if (key == 'y' || key == 'Y'){
                    twist.linear.x = -linear_speed_;
                    twist.angular.z = -angular_speed_;
                } 
                else if (key == 'c' || key == 'C'){
                    twist.linear.x = -linear_speed_;
                    twist.angular.z = angular_speed_;
                } 

                else if (key == 'o' || key == 'O'){
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                }
                
                else if (key == 'x' || key == 'X') {
                    std::cout << "Exiting keyboard controller...\n"; 
                    ros::shutdown();
                    return;
                }
                else{
                    std::cout << "Invalid key: '" << key << "'. Use WASD keys.\n";
                    return;
                }
                // Update the last command time and publish the message
                last_command_time_ = std::chrono::steady_clock::now();
                publishMessage(twist);
            }
        }

        // Publish a TwistStamped message and log it to the console
        void publishMessage(const geometry_msgs::Twist& twist){
            twist_pub_.publish(twist);
            ROS_INFO_STREAM("Published Twist message - Linear: " << twist.linear.x << ", Angular: " << twist.angular.z);
        }

        // Stop the robot by publishing a zero-velocity message
        void stopRobot(){
            geometry_msgs::Twist twist;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            publishMessage(twist);
            ROS_WARN("No command received for a while. Stopping the robot.");
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