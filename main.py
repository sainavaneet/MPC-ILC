import rospy
import numpy as np
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.optimize import minimize
import matplotlib.pyplot as plt

class TurtlebotController:
    def __init__(self):
        self.robot1_x = 0.3
        self.robot1_y = 0.0
        self.robot1_theta = 0.0
        self.robot2_x = 0.0
        self.robot2_y = 0.0
        self.robot2_theta = 0.0
        self.state_leader = None
        self.state_follower = None
        self.robot1_positions = []
        self.robot2_positions = []
        self.control_inputs = []
        self.errors = []
        self.use_ilc = True

        self.L = np.diag([0.1, 0.1])

        self.previous_u = np.zeros(2)
        self.previous_error = np.zeros(3)
        self.L_gain = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])

        rospy.init_node('mpc_follower', anonymous=True)
        self.pub_follower = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
        self.pub_leader = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/robot1/odom', Odometry, self.callback_leader)
        rospy.Subscriber('/robot2/odom', Odometry, self.callback_follower)
        self.rate = rospy.Rate(10)

    def callback_leader(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.robot1_x, self.robot1_y, self.robot1_theta = pos.x, pos.y, yaw
        self.state_leader = np.array([self.robot1_x, self.robot1_y, self.robot1_theta])
        self.robot1_positions.append((self.robot1_x, self.robot1_y))

    def callback_follower(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.robot2_x, self.robot2_y, self.robot2_theta = pos.x, pos.y, yaw
        self.state_follower = np.array([self.robot2_x, self.robot2_y, self.robot2_theta])
        self.robot2_positions.append((self.robot2_x, self.robot2_y))

    def calculate_error(self, state_follower, state_leader):
        error_x = state_follower[0] - state_leader[0]
        error_y = state_follower[1] - state_leader[1]
        error_theta = state_follower[2] - state_leader[2]
        return np.array([error_x, error_y, np.arctan2(np.sin(error_theta), np.cos(error_theta))])

    def toggle_control_strategy(self):
        self.use_ilc = not self.use_ilc

    def update_ilc(self, u_optimal, error_current):
     
        positional_error = error_current[:2] 
        
        if self.previous_u is not None:
            u_ilc = self.previous_u + self.L @ (positional_error - self.previous_error[:2])
        else:
            u_ilc = u_optimal
        
        self.previous_u = u_optimal  # Update past control with current optimal control, not u_ilc
        self.previous_error = error_current  # Store full error for next iteration
        return u_ilc

    def mpc_follower(self):
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 30:
            vel_cmd_leader = Twist()
            vel_cmd_leader.linear.x = 0.3
            vel_cmd_leader.angular.z = 0.15
            self.pub_leader.publish(vel_cmd_leader)
            if self.state_leader is not None and self.state_follower is not None:
                current_error = self.calculate_error(self.state_follower, self.state_leader)
                self.errors.append(current_error)
                if self.use_ilc:
                    control_input = self.update_ilc(self.previous_u, current_error)
                    print(f"control input{control_input}")
                else:
                    control_input = self.calculate_mpc_control_input()
                self.control_inputs.append(control_input if control_input is not None else [0, 0])
                if control_input is not None:
                    vel_cmd = Twist()
                    vel_cmd.linear.x = control_input[0]
                    vel_cmd.angular.z = control_input[1]
                    self.pub_follower.publish(vel_cmd)
                self.robot1_positions.append((self.robot1_x, self.robot1_y))
                self.robot2_positions.append((self.robot2_x, self.robot2_y))
            self.rate.sleep()
        self.plot_trajectories()


    def calculate_mpc_control_input(self):
        T = 0.2
        N = 20
        u_dim = 2
        v_max = 0.4
        omega_max = np.pi
        distance_behind = 1
        u0 = np.zeros(N * u_dim)
        bounds = [(0, v_max) if i % u_dim == 0 else (-omega_max, omega_max) for i in range(N * u_dim)]
        args = (self.state_follower, self.state_leader, T, distance_behind)
        result = minimize(self.cost_function, u0, args=args, method='L-BFGS-B', bounds=bounds)
        return result.x[:u_dim] if result.success else None

    def cost_function(self, u, *args):
        state_follower, state_leader, T, distance_behind = args
        N = len(u) // 2
        x = state_follower.copy()
        cost = 0
        for k in range(N):
            control = u[2 * k:2 * k + 2]
            theta = x[2]
            x[0] += control[0] * np.cos(theta) * T
            x[1] += control[0] * np.sin(theta) * T
            x[2] += control[1] * T
            desired_position = self.get_desired_position(state_leader, distance_behind)
            position_diff = np.linalg.norm(x[:2] - desired_position)
            orientation_diff = x[2] - state_leader[2]
            orientation_error = np.arctan2(np.sin(orientation_diff), np.cos(orientation_diff))
            control_effort = np.sum(np.square(control))
            cost += 7.0 * np.square(position_diff) + 1.0 * np.square(orientation_error) + control_effort
        return cost
    
    def get_desired_position(self, state_leader, distance_behind):
        distance_behind = 0.3
        desired_x = state_leader[0] - distance_behind * np.cos(state_leader[2]) 
        desired_y = state_leader[1] - distance_behind * np.sin(state_leader[2])
        return np.array([desired_x, desired_y])

    def plot_trajectories(self):
        plt.figure(figsize=(10, 8))
        ax = plt.gca()
        leader_x = [pos[0] for pos in self.robot1_positions]
        leader_y = [pos[1] for pos in self.robot1_positions]
        ax.plot(leader_x, leader_y, 'r-', label='Leader', linewidth=2)
        ax.scatter([leader_x[0]], [leader_y[0]], color='green', s=100, edgecolor='black', zorder=5, label='Start (Leader)')
        follower_x = [pos[0] for pos in self.robot2_positions]
        follower_y = [pos[1] for pos in self.robot2_positions]
        ax.plot(follower_x, follower_y, 'b--', label='Follower', linewidth=2)
        ax.scatter([follower_x[0]], [follower_y[0]], color='yellow', s=100, edgecolor='black', zorder=5, label='Start (Follower)')
        ax.set_xlabel('X position')
        ax.set_ylabel('Y position')
        ax.set_title('Robot Trajectories')
        ax.legend()
        ax.grid(True)
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    try:
        controller = TurtlebotController()
        controller.mpc_follower()
    except rospy.ROSInterruptException:
        pass
