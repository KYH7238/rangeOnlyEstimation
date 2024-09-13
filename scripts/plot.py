import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class PlotTrajectory():
    def __init__(self):
        rospy.Subscriber("/relative_pose", PoseStamped, self.relative_pose_callback)
        rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, self.local_pose_callback)

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.relative_trajectory = []
        self.local_trajectory = []

    def relative_pose_callback(self, msg):
        pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.relative_trajectory.append(pos)
        self.update_plot()

    def local_pose_callback(self, msg):
        pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.local_trajectory.append(pos)
        self.update_plot()

    def update_plot(self):
        self.ax.clear()

        if len(self.relative_trajectory) > 0:
            rel_traj = np.array(self.relative_trajectory)
            if rel_traj.size > 0:
                self.ax.plot(rel_traj[:, 0], rel_traj[:, 1], rel_traj[:, 2], 'r', label='Relative Pose')

        if len(self.local_trajectory) > 0:
            loc_traj = np.array(self.local_trajectory)
            if loc_traj.size > 0:
                self.ax.plot(loc_traj[:, 0], loc_traj[:, 1], loc_traj[:, 2], 'b', label='Drone 2 Position in {W}')

        self.ax.legend()
        plt.draw()  # 화면을 다시 그리기 위해 사용
        plt.pause(0.001)  # 잠시 대기하면서 화면 업데이트

if __name__ == "__main__":
    rospy.init_node("PlotTrajectory")
    plotter = PlotTrajectory()
    plt.show(block=True)
    rospy.spin()
