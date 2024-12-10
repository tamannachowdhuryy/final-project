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
        self.target_distance = rospy.get_param('~target_distance', 0.5)  # Closer distance to Robot1
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.5)  # Max linear speed
        self.angular_speed = rospy.get_param('~angular_speed', 1.0)  # Increased max angular speed
        self.robot1_speed_threshold = 0.01  # Speed threshold to detect if Robot1 is moving

        # Robot1 odometry subscriber
        rospy.Subscriber('/robot1/odom', Odometry, self.robot1_odom_callback)

        # Robot2 velocity publisher
        self.cmd_pub = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)

        # State variables for Robot1
        self.robot1_x = 0.0
        self.robot1_y = 0.0
        self.robot1_theta = 0.0
        self.robot1_linear_speed = 0.0

        # State variables for Robot2
        self.robot2_x = 0.0
        self.robot2_y = 0.0
        self.robot2_theta = 0.0

        # Robot2 control rate
        self.rate = rospy.Rate(10)  # 10 Hz
        self.follow()

    def robot1_odom_callback(self, msg):
        # Extract Robot1's position and orientation
        self.robot1_x = msg.pose.pose.position.x
        self.robot1_y = msg.pose.pose.position.y

        # Extract Robot1's orientation
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot1_theta = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        # Calculate Robot1's linear speed
        linear_vel = msg.twist.twist.linear
        self.robot1_linear_speed = sqrt(linear_vel.x**2 + linear_vel.y**2)

    def follow(self):
        while not rospy.is_shutdown():
            try:
                # Check if Robot1 is moving
                if self.robot1_linear_speed < self.robot1_speed_threshold:
                    # Robot1 is stationary, so Robot2 should stop
                    cmd = Twist()  # No movement
                    self.cmd_pub.publish(cmd)
                    self.rate.sleep()
                    continue

                # Calculate the relative position of Robot1 with respect to Robot2
                dx = self.robot1_x - self.robot2_x
                dy = self.robot1_y - self.robot2_y

                # Compute the distance and angle to Robot1
                distance = sqrt(dx**2 + dy**2)
                angle_to_robot1 = atan2(dy, dx)

                # Compute the difference between Robot2's heading and the angle to Robot1
                angle_error = angle_to_robot1 - self.robot2_theta

                # Normalize the angle error to the range [-pi, pi]
                if angle_error > 3.14:
                    angle_error -= 2 * 3.14
                elif angle_error < -3.14:
                    angle_error += 2 * 3.14

                # Maintain the target distance
                distance_error = distance - self.target_distance

                # Dynamic speed adjustment based on Robot1's speed
                linear_x = min(self.robot1_linear_speed, self.max_linear_speed) * max(0, distance_error)

                # Proportional angular velocity control
                angular_z = self.angular_speed * angle_error

                # Cap velocities to prevent instability
                cmd = Twist()
                cmd.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
                cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, angular_z))

                # Publish velocity command
                self.cmd_pub.publish(cmd)

                # Sleep to maintain the loop rate
                self.rate.sleep()
            except Exception as e:
                rospy.logerr(f"Error in follow loop: {e}")

if __name__ == '__main__':
    try:
        Follower()
    except rospy.ROSInterruptException:
        pass

