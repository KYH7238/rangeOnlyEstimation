import rospy
import numpy as np

class gt_pub:
    def __init__(self):
        rospy.init_node("gt_pub")
        self.imu_data = self.load_imu_data("ego_velocity.txt")
        self.drone_pose_data = self.load_drone_z("drone_pose_6D.txt")

    def load_imu_data(self, filepath):
        data = []
        with open(filepath, 'r') as f:
            for line in f:
                vals = line.strip().split()
                t = float(vals[0])
                vx, vy, vz = float(vals[1]), float(vals[2]), float(vals[3])
                wx, wy, wz = float(vals[4]), float(vals[5]), float(vals[6])
                data.append((t,vx,vy,vz,wx,wy,wz))
        return np.array(data) 
    