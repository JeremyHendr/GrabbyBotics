/*
 * Project: Grabby the warehouse robot
 * Code adapted from : https://automaticaddison.com/how-to-create-an-initial-pose-and-goal-publisher-in-ros/
 * Date: 03.02.2025
 * Description: 
 *      Convert initial pose and goal into usable format (from rviz)
 *      Subscribe:
 *          initialpose : The initial position and orientation of the robot using 
 *                        quaternions. (geometry_msgs/PoseWithCovarianceStamped)
 *          move_base_simple/goal : Goal position and orientation (geometry_msgs::PoseStamped)
 *      Publish: This node publishes to the following topics:   
 *          goal_2d : Goal position and orientation (geometry_msgs::PoseStamped)
 *          initial_2d : The initial position and orientation of the robot using Euler angles. (geometry_msgs/PoseStamped)
 */
 
// Include statements 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <iostream>

class PoseConverter {
    public:
        PoseConverter(){
            // Init ROS publishers
            pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_2d", 0);
            pub_initial_ = nh_.advertise<geometry_msgs::PoseStamped>("initial_2d", 0);

            // Init ROS subscribers
            sub_goal_ = nh_.subscribe("move_base_simple/goal", 0, &PoseConverter::handleGoal, this);
            sub_initial_ = nh_.subscribe("initialpose", 0, &PoseConverter::handleInitialPose, this);
        }

        // Take move_base_simple/goal as input and publish goal_2d
        void handleGoal(const geometry_msgs::PoseStamped &goal){
            geometry_msgs::PoseStamped rpyGoal;
            rpyGoal.header.frame_id = "map";
            rpyGoal.header.stamp = goal.header.stamp;
            rpyGoal.pose.position.x = goal.pose.position.x;
            rpyGoal.pose.position.y = goal.pose.position.y;
            rpyGoal.pose.position.z = 0;

            tf::Quaternion q(0, 0, goal.pose.orientation.z, goal.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            rpyGoal.pose.orientation.x = 0;
            rpyGoal.pose.orientation.y = 0;
            rpyGoal.pose.orientation.z = yaw;
            rpyGoal.pose.orientation.w = 0;

            pub_goal_.publish(rpyGoal);
        }

        // Take initialpose as input and publish initial_2d
        void handleInitialPose(const geometry_msgs::PoseWithCovarianceStamped &pose){
            geometry_msgs::PoseStamped rpyPose;
            rpyPose.header.frame_id = "map";
            rpyPose.header.stamp = pose.header.stamp;
            rpyPose.pose.position.x = pose.pose.pose.position.x;
            rpyPose.pose.position.y = pose.pose.pose.position.y;
            rpyPose.pose.position.z = 0;

            tf::Quaternion q(0, 0, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            rpyPose.pose.orientation.x = 0;
            rpyPose.pose.orientation.y = 0;
            rpyPose.pose.orientation.z = yaw;
            rpyPose.pose.orientation.w = 0;

            pub_initial_.publish(rpyPose);

        }

        void spin(){
            ros::Rate loop_rate(10);
            while (ros::ok()){
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_goal_;
        ros::Publisher pub_initial_;
        ros::Subscriber sub_goal_;
        ros::Subscriber sub_initial_;
};

int main (int argc, char* argv[]){
    ros::init(argc, argv, "rviz_click_to_2d");
    PoseConverter poseConverter;
    poseConverter.spin();
    return 0;
}