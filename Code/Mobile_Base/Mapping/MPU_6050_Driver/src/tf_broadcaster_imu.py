#!/usr/bin/env python2

# ------------------------ #
# Project: Grabby the Warehouse robot
# Date: 15.01.2025
# Description: Code found on GitHub at the link below. Transforms IMU data to visualize it in RVIZ
# Link: https://github.com/bandasaikrishna/orientations_from_IMU_MPU_6050/blob/main/mpu_6050_driver/scripts/tf_broadcaster_imu.py
# ------------------------ #

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu


def handle_imu_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "imu_link"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z
    t.transform.rotation.w = msg.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
      rospy.init_node('tf_broadcaster_imu')
      rospy.Subscriber('/imu', Imu, handle_imu_pose)
      rospy.spin()