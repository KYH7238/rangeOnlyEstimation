import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from gazebo_msgs.msg import ModelStates
import numpy as np
import tf
from matplotlib.patches import Polygon

class PlotTrajectory():
    def __init__(self):
        self.relative_trajectory = []
        self.local_trajectory = []
        self.triangles = []
        self.rel_counter = 0
        self.loc_counter = 0
        self.cnt = 0
        self.drone_x = 0
        self.drone_y = 0
        self.drone_yaw = 0
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/estimated_state", PoseStamped, self.relative_pose_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.local_pose_callback)
        self.fig, self.ax = plt.subplots()
        self.wRi = np.eye(3)

    def pose_cb(self, msg):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.wRi = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.drone_yaw = euler[2]

    def relative_pose_callback(self, msg):
        self.rel_counter += 1
        if self.rel_counter % 3 == 0:
            pos_vector = np.array([msg.pose.position.x, msg.pose.position.y, 0.11])
            pos_world = np.dot(self.wRi, pos_vector) + np.array([self.drone_x, self.drone_y, 0])
            yaw_angle = self.drone_yaw + msg.pose.position.z
            self.relative_trajectory.append((pos_world[0], pos_world[1], yaw_angle))
            self.cnt += 1
            if self.cnt >= 5:
                self.triangles.append((pos_world[0], pos_world[1], yaw_angle))
                self.cnt = 0

    def local_pose_callback(self, msg):
        self.loc_counter += 1
        if self.loc_counter % 3 == 0:
            index = msg.name.index('jackal')
            pose = msg.pose[index]
            pos = (pose.position.x, pose.position.y)
            self.local_trajectory.append(pos)

    def create_rotated_triangle(self, x, y, yaw, size=0.08):
        triangle = np.array([
            [size*2.5, 0],
            [0, -size],
            [0, size]
        ])
        rot = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])
        triangle_rotated = np.dot(rot,triangle.T)
        triangle_rotated[0,:] += x
        triangle_rotated[1,:] += y
        return Polygon(triangle_rotated.T, color='r', fill=False)

    def animate(self, frame):
        self.ax.clear()
        self.ax.set_title("Global Frame")
        if self.relative_trajectory:
            rel_traj = np.array(self.relative_trajectory)
            self.ax.plot(rel_traj[:, 0], rel_traj[:, 1], 'r-', label='Estimated')
            if self.triangles:
                for x, y, yaw in self.triangles:
                    triangle = self.create_rotated_triangle(x, y, yaw)
                    self.ax.add_patch(triangle)
        if self.local_trajectory:
            loc_traj = np.array(self.local_trajectory)
            self.ax.scatter(loc_traj[:, 0], loc_traj[:, 1], c='k', label='Ground Truth')
        self.ax.legend()

if __name__ == "__main__":
    rospy.init_node("plot_node")
    plotter = PlotTrajectory()
    ani = FuncAnimation(plotter.fig, plotter.animate, interval=100)
    plt.show(block=True)
    