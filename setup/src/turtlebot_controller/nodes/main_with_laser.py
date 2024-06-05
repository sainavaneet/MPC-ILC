#!/usr/bin/env python3


#Load Velocities and simulate Turtlebot
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import LaserScan
import numpy as np

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
        self.cmd_vel_robot2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_robot1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/robot2/scan', LaserScan, self.scan_callback)

        self.robot1_x = 0.0
        self.robot1_y = 0.0
        self.robot1_theta = 0.0
        self.robot2_x = -1.0
        self.robot2_y = 0.0
        self.robot2_theta = 0.0

        self.k_x = 0.5
        self.k_theta = 0.5
        self.robot1_positions = []
        self.robot2_positions = []

        self.fig = plt.figure(figsize=(10, 8))
        self.fig.suptitle('Leader - Follower Tracking', fontsize=16)
        self.ax = self.fig.add_subplot(111, projection='3d', facecolor=(1,1,1))

        self.velocities = self.load_velocities()

    def load_velocities(self):
        velocities = []
        with open("nodes/velocities.txt", "r") as f:
            next(f) 
            for line in f:
                try:
                    v, omega = line.split(',')
                    velocities.append((float(v), float(omega)))
                except ValueError:
                    print(f"Skipping malformed line: {line.strip()}")
        return velocities



    def scan_callback(self, msg):
        threshold = 0.5
        clusters = []
        cluster = []
        angles = []

        for i in range(len(msg.ranges)):
            if msg.ranges[i] == float('inf'):  
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if cluster:
                last_angle = angles[-1]
                angle_diff = angle - last_angle
                if angle_diff < msg.angle_increment + 1e-5 and abs(msg.ranges[i] - cluster[-1]) < threshold:
                    cluster.append(msg.ranges[i])
                    angles.append(angle)
                else:
                    clusters.append((cluster, angles))
                    cluster = [msg.ranges[i]]
                    angles = [angle]
            else:
                cluster = [msg.ranges[i]]
                angles = [angle]

        if cluster:  
            clusters.append((cluster, angles))

        clusters.sort(key=lambda x: len(x[0]), reverse=True)

        if clusters:
            distance_to_robot = np.mean(clusters[0][0])
            angle_to_robot = np.mean(clusters[0][1])
            
            self.robot1_x = distance_to_robot * np.cos(angle_to_robot) - 1
            self.robot1_y = distance_to_robot * np.sin(angle_to_robot)
            self.robot1_theta = np.arctan2(self.robot1_y, self.robot1_x)
            matrix_format = """
            robot1:
            [ {:.5f} 
            {:.5f} 
            {:.5f} ] rad
            """.format(self.robot1_x, self.robot1_y, self.robot1_theta)

            rospy.loginfo(matrix_format)
        else:
            rospy.loginfo("Robot1 not in range")


    def control_loop(self):
        rate = rospy.Rate(10)
        velocity_index = 0
        while not rospy.is_shutdown() and velocity_index < len(self.velocities):
            x1 = self.robot1_x
            y1 = self.robot1_y
            theta1 = self.robot1_theta
            
            x_error_12 = x1 - self.robot2_x
            y_error_12 = y1 - self.robot2_y
            theta_error_12 = theta1 - self.robot2_theta

            linear_vel_robot2 = self.k_x * math.sqrt(x_error_12**2 + y_error_12**2)
            angular_vel_robot2 = self.k_theta * np.arctan2(y_error_12 , x_error_12)

            # Publish velocities for robot1
            control_command_robot1 = Twist()
            control_command_robot1.linear.x = self.velocities[velocity_index][0]
            control_command_robot1.angular.z = self.velocities[velocity_index][1]
            self.cmd_vel_robot1.publish(control_command_robot1)

            velocity_index += 1

            control_command_robot2 = Twist()
            control_command_robot2.linear.x = linear_vel_robot2
            control_command_robot2.angular.z = angular_vel_robot2
            self.cmd_vel_robot2.publish(control_command_robot2)

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.control_loop()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Ctrl+C pressed. Stopping robots.")
        pass
