import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from gazebo_msgs.msg import ModelStates
import numpy as np
import message_filters
import tf

class PlotTrajectory():
    def __init__(self):
        rospy.init_node("plot_node")
        self.relative_trajectory = []
        self.local_trajectory = []

        # Subscribers with message_filters
        self.est_sub = message_filters.Subscriber("/estimated_state", PoseStamped)
        self.gt_sub = message_filters.Subscriber("/gazebo/model_states", ModelStates)
        self.uav_sub = message_filters.Subscriber("/mavros/local_position/pose", PoseStamped)

        # Time Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([self.est_sub, self.gt_sub, self.uav_sub], 10, 0.1)
        ts.registerCallback(self.synchronized_callback)

        self.fig, self.ax = plt.subplots()

    def synchronized_callback(self, est_msg, gt_msg, uav_msg):
        # UAV 위치
        uav_x = uav_msg.pose.position.x
        uav_y = uav_msg.pose.position.y

        # UGV 위치 (Ground Truth)
        try:
            index = gt_msg.name.index('jackal')
        except ValueError:
            rospy.logwarn("Jackal model not found in ModelStates")
            return
        ugv_pose = gt_msg.pose[index]
        ugv_x = ugv_pose.position.x
        ugv_y = ugv_pose.position.y

        # 상대 위치 계산 (Ground Truth)
        rel_x = ugv_x - uav_x
        rel_y = ugv_y - uav_y
        self.local_trajectory.append((rel_x, rel_y))

        # 추정된 상대 위치
        est_rel_x = est_msg.pose.position.x
        est_rel_y = est_msg.pose.position.y
        self.relative_trajectory.append((est_rel_x, est_rel_y))

    def animate(self, frame):
        self.ax.clear()
        if self.relative_trajectory:
            rel_traj = np.array(self.relative_trajectory)
            self.ax.plot(rel_traj[:, 0], rel_traj[:, 1], c='r', label='Estimated')
        if self.local_trajectory:
            loc_traj = np.array(self.local_trajectory)
            self.ax.plot(loc_traj[:, 0], loc_traj[:, 1], c='b', label='Ground Truth')
        self.ax.legend()
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Relative Trajectory')
        self.ax.grid(True)

    def run(self):
        ani = FuncAnimation(self.fig, self.animate, interval=100)
        plt.show(block=True)

if __name__ == "__main__":
    plotter = PlotTrajectory()
    plotter.run()
