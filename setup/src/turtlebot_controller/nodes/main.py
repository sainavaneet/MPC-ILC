# This contain leader follower of turtlebot with State Feedback Linearization.

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
        rospy.Subscriber('/robot1/odom', Odometry, self.robot1_odom_callback)
        rospy.Subscriber('/robot2/odom', Odometry, self.robot2_odom_callback)
        # rospy.Subscriber('/robot3/odom', Odometry, self.robot3_odom_callback)
        
        self.cmd_vel_robot2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_robot1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        # self.cmd_vel_robot3 = rospy.Publisher('/robot3/cmd_vel', Twist, queue_size=10)

        self.robot1_x = 0.0
        self.robot1_y = 0.0
        self.robot1_theta = 0.0
        self.robot2_x = -1.0
        self.robot2_y = 0.0
        self.robot2_theta = 0.0
        # self.robot3_x = -2.0
        # self.robot3_y = 0.0
        # self.robot3_theta = 0.0

        self.k_x =  0.5
        self.k_theta = 0.5
        self.robot1_positions = []
        self.robot2_positions = []
        # self.robot3_positions = []

        self.fig = plt.figure(figsize=(10, 8))
        self.fig.suptitle('Leader - Follower Tracking', fontsize=16)
        self.ax = self.fig.add_subplot(111, projection='3d' , facecolor=(1,1,1))
        

    def robot1_odom_callback(self, msg):
        self.robot1_x = msg.pose.pose.position.x
        self.robot1_y = msg.pose.pose.position.y
        orientation_quaternion = msg.pose.pose.orientation
        (_, _, self.robot1_theta) = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])

    def robot2_odom_callback(self, msg):
        self.robot2_x = msg.pose.pose.position.x
        self.robot2_y = msg.pose.pose.position.y
        orientation_quaternion = msg.pose.pose.orientation
        (_, _, self.robot2_theta) = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])


    
    def control_loop(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            x1 = self.robot1_x 
            y1 = self.robot1_y 
            theta1 = self.robot1_theta
            
            x_error_12 = x1 - self.robot2_x
            y_error_12 = y1 - self.robot2_y
            theta_error_12 = theta1 - self.robot2_theta

            
            #normalizing the theta 
            if theta_error_12 > math.pi:
                theta_error_12 -= 2.0 * math.pi
            elif theta_error_12 < -math.pi:
                theta_error_12 += 2.0 * math.pi


            linear_vel_robot2 = self.k_x * math.sqrt(x_error_12**2 + y_error_12**2)

            angular_vel_robot2 = self.k_theta * theta_error_12

  


            self.robot1_positions.append((x1, y1, 0))
            self.robot2_positions.append((self.robot2_x, self.robot2_y, 0))

            self.ax.clear()
            self.ax.plot3D(*zip(*self.robot1_positions), label='Robot 1', c='r', markersize ='0.5',marker='o')
            self.ax.plot3D(*zip(*self.robot2_positions), label='Robot 2', c='g', markersize ='0.5',marker='o')

            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')
            self.ax.set_xlim(2, -2)  
            self.ax.set_ylim(2, -2)
            self.ax.set_zlim(3, -3)

            self.ax.legend(fontsize=20)

            plt.pause(0.01)

            control_command_robot1 = Twist()
            control_command_robot1.linear.x = 0.2
            control_command_robot1.angular.z = 0.2  
            self.cmd_vel_robot1.publish(control_command_robot1)

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
