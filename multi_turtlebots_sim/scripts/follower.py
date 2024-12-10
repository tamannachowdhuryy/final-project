#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

class Follower:
    def __init__(self):
        rospy.init_node('robot2_follower', anonymous=True)

        # Parameters
        self.target_distance = rospy.get_param('~target_distance', 0.5)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.5)
        self.angular_speed = rospy.get_param('~angular_speed', 1.0)
        self.robot1_speed_threshold = 0.01

        # Subscribers
        rospy.Subscriber('/robot1/odom', Odometry, self.robot1_odom_callback)
        rospy.Subscriber('/robot2/odom', Odometry, self.robot2_odom_callback)

        # Publisher
        self.cmd_pub = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)

        # State variables
        self.robot1_x = self.robot1_y = self.robot1_theta = 0.0
        self.robot1_linear_speed = 0.0
        self.robot2_x = self.robot2_y = self.robot2_theta = 0.0

        # Control rate
        self.rate = rospy.Rate(10)
        self.follow()

    def robot1_odom_callback(self, msg):
        self.robot1_x = msg.pose.pose.position.x
        self.robot1_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot1_theta = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        linear_vel = msg.twist.twist.linear
        self.robot1_linear_speed = sqrt(linear_vel.x**2 + linear_vel.y**2)

    def robot2_odom_callback(self, msg):
        self.robot2_x = msg.pose.pose.position.x
        self.robot2_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot2_theta = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

    def follow(self):
        while not rospy.is_shutdown():
            try:
                dx = self.robot1_x - self.robot2_x
                dy = self.robot1_y - self.robot2_y
                distance = sqrt(dx**2 + dy**2)
                angle_to_robot1 = atan2(dy, dx)
                angle_error = angle_to_robot1 - self.robot2_theta

                # Normalize angle error
                angle_error = (angle_error + 3.14) % (2 * 3.14) - 3.14

                distance_error = distance - self.target_distance

                # Adjust linear and angular speed
                linear_x = max(0, min(self.max_linear_speed, self.robot1_linear_speed * distance_error))
                angular_z = self.angular_speed * angle_error * (1 if abs(distance_error) > 0.1 else 0.5)

                # Stop if Robot1 is stationary
                if self.robot1_linear_speed < self.robot1_speed_threshold and distance_error < 0.1:
                    linear_x = 0
                    angular_z = 0

                # Publish command
                cmd = Twist()
                cmd.linear.x = linear_x
                cmd.angular.z = angular_z
                self.cmd_pub.publish(cmd)

                self.rate.sleep()
            except Exception as e:
                rospy.logerr(f"Error in follow loop: {e}")

if __name__ == '__main__':
    try:
        Follower()
    except rospy.ROSInterruptException:
        pass


