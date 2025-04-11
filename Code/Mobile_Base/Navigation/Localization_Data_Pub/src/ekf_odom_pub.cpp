/*
 * Project: Grabby the warehouse robot
 * Date: 06.02.2025
 * Adapted from: https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/
 * Description: Publishes odometry information created with wheel encoder ticks counts.
 *              Subscribe: 
 *                m1_ticks : Tick counts from the m1 motor encoder (std_msgs/Int16)
 *                m2_ticks : Tick counts from the m2 motor encoder (std_msgs/Int16)
 *                initial_2d : The initial position and orientation of the robot (geometry_msgs/PoseStamped)
 *              Publish: 
 *                odom_data_euler : Position and velocity estimate. The orientation.z represents the yaw angle (nav_msgs/Odometry)
 *                odom_data_quat : Position and velocity estimate. The orientation is in quaternion format (nav_msgs/Odometry)
 */

// Include various libraries
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
 
class OdomPublisher {
    public:
        OdomPublisher() {
            // Init publishers
            odom_data_pub = nh.advertise<nav_msgs::Odometry>("odom_data_euler", 100);
            odom_data_pub_quat = nh.advertise<nav_msgs::Odometry>("odom", 100);

            // Init subscribers
            sub_ini_pose = nh.subscribe("initial_2d", 1, &OdomPublisher::set_initial_2d, this);
            sub_m1_ticks = nh.subscribe("m1_ticks", 100, &OdomPublisher::calc_m1, this);
            sub_m2_ticks = nh.subscribe("m2_ticks", 100, &OdomPublisher::calc_m2, this);
        
            // Init odometry messages
            odomNew.header.frame_id = "odom";
            odomNew.pose.pose.position.z = 0;
            odomNew.pose.pose.orientation.x = 0;
            odomNew.pose.pose.orientation.y = 0;
            odomNew.twist.twist.linear.x = 0;
            odomNew.twist.twist.linear.y = 0;
            odomNew.twist.twist.linear.z = 0;
            odomNew.twist.twist.angular.x = 0;
            odomNew.twist.twist.angular.y = 0;
            odomNew.twist.twist.angular.z = 0;
            
            odomOld.pose.pose.position.x = initialX;
            odomOld.pose.pose.position.y = initialY;
            odomOld.pose.pose.orientation.z = initialTheta;
        }

        
        void run(){
            ros::Rate loop_rate(30);
            while (ros::ok()){
                if (initialPoseReceived){
                    update_odom();
                    publish_quat();
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher odom_data_pub, odom_data_pub_quat;
        ros::Subscriber sub_ini_pose, sub_m1_ticks, sub_m2_ticks;

        nav_msgs::Odometry odomNew, odomOld;
        bool initialPoseReceived = false;
        double distanceM1 = 0, distanceM2 = 0;

        const double initialX = 0.0, initialY = 0.0, initialTheta = 1e-10;
        const double PI = 3.141592;
        const double WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
        const double WHEEL_BASE = 0.18;
        const double WHEEL_RADIUS = 0.035;
        const double GEAR_RATIO = 98.78;
        const double MOTOR_SHAFT_TICKS = 12;
        const double GEAR_BOX_SHAFT_TICKS = GEAR_RATIO * MOTOR_SHAFT_TICKS;
        const double TICKS_PER_REVOLUTION = GEAR_BOX_SHAFT_TICKS;
        const double TICKS_PER_METER = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE; // 5393.45 ticks per meter

        // Get the initial 2D position from RVIZ
        void set_initial_2d (const geometry_msgs::PoseStamped &rvizClick){
            odomOld.pose.pose.position.x = rvizClick.pose.position.x;
            odomOld.pose.pose.position.y = rvizClick.pose.position.y;
            odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
            initialPoseReceived = true;
        }

        // Compute the distance the wheel on M1 has travelled
        void calc_m1 (const std_msgs::Int16& m1Count){
            static int lastCountM1 = 0;
            if (m1Count.data != 0 && lastCountM1 != 0){
                int m1Ticks = m1Count.data - lastCountM1;
                
                if (m1Ticks > 10000){
                    m1Ticks = 0 - (65535 - m1Ticks);
                }
                else if (m1Ticks < -10000){
                    m1Ticks = 65535 - m1Ticks;
                }
                else{}
                distanceM1 = m1Ticks / TICKS_PER_METER;
            }
            lastCountM1 = m1Count.data;
        }

        // Compute the distance the wheel on M2 has travelled
        void calc_m2 (const std_msgs::Int16& m2Count){
            static int lastCountM2 = 0;
            if (m2Count.data != 0 && lastCountM2 != 0){
                int m2Ticks = m2Count.data - lastCountM2;
                
                if (m2Ticks > 10000){
                    m2Ticks = 0 - (65535 - m2Ticks);
                }
                else if (m2Ticks < -10000){
                    m2Ticks = 65535 - m2Ticks;
                }
                else{}
                distanceM2 = m2Ticks / TICKS_PER_METER;
            }
            lastCountM2 = m2Count.data;
        }

        // Publish a nav_msgs::Odometry message in quaternion format
        void publish_quat() {
            tf2::Quaternion q;
            
            q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

            nav_msgs::Odometry quatOdom;
            quatOdom.header.stamp = odomNew.header.stamp;
            quatOdom.header.frame_id = "odom";
            quatOdom.child_frame_id = "base_link";
            quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
            quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
            quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
            quatOdom.pose.pose.orientation.x = q.x();
            quatOdom.pose.pose.orientation.y = q.y();
            quatOdom.pose.pose.orientation.z = q.z();
            quatOdom.pose.pose.orientation.w = q.w();
            quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
            quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
            quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
            quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
            quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
            quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
            
            for(int i = 0; i<36; i++) {
                if(i == 0 || i == 7 || i == 14) {
                    quatOdom.pose.covariance[i] = .01;
                }
                else if (i == 21 || i == 28 || i== 35) {
                    quatOdom.pose.covariance[i] += 0.1;
                }
                else {
                    quatOdom.pose.covariance[i] = 0;
                }
            }

            odom_data_pub_quat.publish(quatOdom);
        }

        // Update odometry information
        void update_odom(){
            // Calculate the average distance
            double cycleDistance = (distanceM1 + distanceM2) / 2;
            
            // Calculate the number of radians the robot has turned since the last cycle
            double cycleAngle = asin((distanceM2 - distanceM1) / WHEEL_BASE);
            
            // Average angle during the last cycle
            double avgAngle = (cycleAngle / 2) + odomOld.pose.pose.orientation.z;

            if (avgAngle > PI) {
                avgAngle -= 2*PI;
            }
            else if (avgAngle < -PI) {
                avgAngle += 2*PI;
            }
            else{}
            
            // Calculate the new pose (x, y, and theta)
            odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
            odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
            odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

            // Prevent lockup from a single bad cycle
            if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y) || isnan(odomNew.pose.pose.position.z)) {
                odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
                odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
                odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
            }

            // Normalize the theta angle
            if (odomNew.pose.pose.orientation.z > PI) {
                odomNew.pose.pose.orientation.z -= 2 * PI;
            }
            else if (odomNew.pose.pose.orientation.z < -PI) {
                odomNew.pose.pose.orientation.z += 2 * PI;
            }
            else{}
        
            // Compute the velocity
            odomNew.header.stamp = ros::Time::now();
            odomNew.twist.twist.linear.x = cycleDistance / (odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
            odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
            
            // Save the pose data for the next cycle
            odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
            odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
            odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
            odomOld.header.stamp = odomNew.header.stamp;

            // Publish the odometry message
            odom_data_pub.publish(odomNew);
        }
};

int main (int argc, char **argv){
    ros::init(argc, argv, "ekf_odom_pub");
    OdomPublisher odom_publisher;
    odom_publisher.run();
    return 0;
}


