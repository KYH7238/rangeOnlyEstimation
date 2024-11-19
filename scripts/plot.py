import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from gazebo_msgs.msg import ModelStates
import numpy as np
import tf

class PlotTrajectory():
    def __init__(self):
        self.relative_trajectory = []
        self.local_trajectory = []
        self.rel_counter = 0
        self.loc_counter = 0
        self.drone_x, self.drone_y = 0, 0
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

    def relative_pose_callback(self, msg):
        self.rel_counter += 1
        if self.rel_counter % 3 == 0:
            pos_vector = np.array([msg.pose.position.x, msg.pose.position.y, 0.11])
            pos_world = np.dot(self.wRi, pos_vector) + np.array([self.drone_x, self.drone_y, 0])
            self.relative_trajectory.append((pos_world[0], pos_world[1]))

    def local_pose_callback(self, msg):
        self.loc_counter += 1
        if self.loc_counter % 3 == 0:
            index = msg.name.index('jackal')
            pose = msg.pose[index]
            pos = (pose.position.x, pose.position.y)
            self.local_trajectory.append(pos)

    def animate(self, frame):
        self.ax.clear()
        if self.relative_trajectory:
            rel_traj = np.array(self.relative_trajectory)
            self.ax.scatter(rel_traj[:, 0], rel_traj[:, 1], c='r', label='Estimated')
        if self.local_trajectory:
            loc_traj = np.array(self.local_trajectory)
            self.ax.scatter(loc_traj[:, 0], loc_traj[:, 1], c='b', label='Ground Truth')
        self.ax.legend()

if __name__ == "__main__":
    rospy.init_node("plot_node")
    plotter = PlotTrajectory()
    ani = FuncAnimation(plotter.fig, plotter.animate, interval=100)
    plt.show(block=True)
