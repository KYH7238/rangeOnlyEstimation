
import rospy
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from gazebo_msgs.msg import ModelStates
import numpy as np

class PlotTrajectory():
    def __init__(self):
        # Initialize attributes
        self.relative_trajectory = []
        self.local_trajectory = []
        self.rel_counter = 0
        self.loc_counter = 0

        # Subscribers
        rospy.Subscriber("/estimated_state", Pose2D, self.relative_pose_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.local_pose_callback)

        # Set up the plot
        self.fig, self.ax = plt.subplots()

    def relative_pose_callback(self, msg):
        self.rel_counter += 1
        if self.rel_counter % 3 == 0:
            pos = (msg.x, msg.y)
            self.relative_trajectory.append(pos)

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
