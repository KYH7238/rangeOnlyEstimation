import rospy
import numpy as np
from nlink_parser.msg import LinktrackAnchorframe0
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import bisect
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class hardware_rpe:
    def __init__(self):
        self.state = {"xji":0.0, "yji":0.0, "thetaji":0.0, "vj":0.0, "wj":0.0}
        self.matP = np.eye(5)*0.01
        self.matQ = np.eye(5)*0.001
        self.matR = np.eye(8)*0.015

        self.jacobianF = np.eye(5)
        self.jacobianH = np.zeros((8,5))
        self.vecZ = np.zeros(8)
        self.vecH = np.zeros(8)

        self.uavUwbPositions = np.array([
            [0.28,    0.0,    0.0],
            [0.0,    -0.28,   0.0],  
            [-0.28,   0.0,    0.0],  
            [0.0,     0.28,   0.0]   
        ])
        self.ugvUwbPositions = np.array([
            [0.335, 0.0, 0.0],
            [-0.335,0.0, 0.0]
        ])

        self.vi = np.zeros(3)
        self.wi = np.zeros(3)
        self.dt = 0.0
        self.droneZ = 1.0
        self.pub = rospy.Publisher("/ekf_result", PoseStamped, queue_size=10)

    def setHeight(self, z_):
        self.droneZ = z_

    def setViWi(self, vi, wi):
        self.vi = vi
        self.wi = wi

    def setZ(self, z):
        for i in range(z.size):
            horizontal_dist = np.sqrt(z[i]**2 - (self.droneZ - 0.22)**2)
            self.vecZ[i] = horizontal_dist

        self.vecH = np.zeros(8)
        self.jacobianH = np.zeros((8,5))

    def setDt(self, dt):
        self.dt = dt

    def motionModel(self):
        xji = self.state["xji"]
        yji = self.state["yji"]
        theta = self.state["thetaji"]
        vj = self.state["vj"]
        wj = self.state["wj"]
        vi = self.vi
        wi = self.wi

        new_xji = xji + (vj*np.cos(theta) + yji*wi[2] - vi[0])*self.dt
        new_yji = yji + (vj*np.sin(theta) - xji*wi[2])*self.dt
        new_theta = theta + (wj - wi[2])*self.dt

        self.state["xji"] = new_xji
        self.state["yji"] = new_yji
        self.state["thetaji"] = new_theta

    def motionModelJacobian(self):
        theta = self.state["thetaji"]
        vj = self.state["vj"]
        wi = self.wi
        dt = self.dt

        F = np.eye(5)
        F[0,1] = wi[2]*dt
        F[0,2] = -vj*np.sin(theta)*dt
        F[0,3] = np.cos(theta)*dt

        F[1,0] = -wi[2]*dt
        F[1,2] = vj*np.cos(theta)*dt
        F[1,3] = np.sin(theta)*dt

        F[2,4] = dt

        self.jacobianF = F

    def prediction(self):
        self.motionModel()
        self.motionModelJacobian()
        self.matP = self.jacobianF @ self.matP @ self.jacobianF.T + self.matQ

    def measurementModelJacobian(self, pi, pj, pjInUav):
        theta = self.state["thetaji"]
        dist = np.linalg.norm(pi - pjInUav)
        if dist < 1e-4:
            dist = 1e-4
        dx = pjInUav[0]-pi[0]
        dy = pjInUav[1]-pi[1]

        dH = np.zeros(5)
        dH[0] = dx/dist
        dH[1] = dy/dist
        dH[2] = ((-np.sin(theta)*pj[0]-np.cos(theta)*pj[1])*dx +
                 (np.cos(theta)*pj[0]-np.sin(theta)*pj[1])*dy)/dist

        return dH

    def measurementModel(self):
        theta = self.state["thetaji"]
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,0,1]
        ])
        relPose = np.array([self.state["xji"], self.state["yji"], 0])
        idx = 0

        for uav_idx in [0, 1, 2, 3]: 
            pi = self.uavUwbPositions[uav_idx, :]
            pj = self.ugvUwbPositions[0, :] 
            pjInUav = R @ pj + relPose
            dist = np.linalg.norm(pi - pjInUav)
            self.vecH[idx] = dist
            self.jacobianH[idx, :] = self.measurementModelJacobian(pi, pj, pjInUav)
            idx += 1

        for uav_idx in [0, 1, 2, 3]:
            pi = self.uavUwbPositions[uav_idx, :]
            pj = self.ugvUwbPositions[1, :]  
            pjInUav = R @ pj + relPose
            dist = np.linalg.norm(pi - pjInUav)
            self.vecH[idx] = dist
            self.jacobianH[idx, :] = self.measurementModelJacobian(pi, pj, pjInUav)
            idx += 1

    def correction(self):
        self.measurementModel()
        residual = self.vecZ - self.vecH
        S = self.jacobianH @ self.matP @ self.jacobianH.T + self.matR
        K = self.matP @ self.jacobianH.T @ np.linalg.inv(S)

        stateVec = np.array([
            self.state["xji"], 
            self.state["yji"], 
            self.state["thetaji"], 
            self.state["vj"], 
            self.state["wj"]
        ])
        stateVec = stateVec + K @ residual

        self.state["xji"]    = stateVec[0]
        self.state["yji"]    = stateVec[1]
        self.state["thetaji"]= stateVec[2]
        self.state["vj"]     = stateVec[3]
        self.state["wj"]     = stateVec[4]

        I = np.eye(5)
        self.matP = (I - K @ self.jacobianH) @ self.matP

    def pub_(self, data):
        pose = PoseStamped()
        pose.header.frame_id = "ekf"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = data[0]
        pose.pose.position.y = data[1]
        pose.pose.position.z = data[2]
        self.pub.publish(pose)

class EKFNode:
    def __init__(self):
        rospy.init_node("relative_pose_estimation")
        self.ekf_ = hardware_rpe()
        self.imu_data = self.load_imu_data("ego_velocity.txt")
        self.drone_times, self.drone_x, self.drone_y, self.drone_z, self.wRi_list = self.load_drone_pose("drone_pose_6D.txt")
        self.uwb_init = False
        self.uwb_init_time = 0.0
        self.prevTime_ = None
        self.ekf_x = []
        self.ekf_y = []
        self.uwb_sub = rospy.Subscriber("/nlink_linktrack_anchorframe0", LinktrackAnchorframe0, self.uwbCallback, queue_size=1)
        self.ekf_sub = rospy.Subscriber("/ekf_result", PoseStamped, self.ekf_result_cb)
        self.hunter_x = []
        self.hunter_y = []
        self.read_gt()
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_title("EKF vs Hunter GT")

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
    
    def ekf_result_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.ekf_x.append(x)
        self.ekf_y.append(y)

    def load_drone_pose(self, filepath):
        times = []
        xs = []
        ys = []
        zs = []
        wRi_list = []
        with open(filepath, 'r') as f:
            for line in f:
                vals = line.strip().split()
                t = float(vals[0])
                x_m = float(vals[1])/1000.0
                y_m = float(vals[2])/1000.0
                z_m = float(vals[3])/1000.0
                Rot=list(map(float,vals[8:17]))
                wRi = np.array(Rot).reshape(3,3)
                times.append(t)
                xs.append(x_m)
                ys.append(y_m)
                zs.append(z_m)
                wRi_list.append(wRi)
        return np.array(times), np.array(xs), np.array(ys), np.array(zs), wRi_list

    def interpolate_imu(self, t):
        times = self.imu_data[:,0]
        idx = np.searchsorted(times, t)
        if idx == 0:
            vx,vy,vz = self.imu_data[0,1:4]
            wx,wy,wz = self.imu_data[0,4:7]
        elif idx >= len(times):
            vx,vy,vz = self.imu_data[-1,1:4]
            wx,wy,wz = self.imu_data[-1,4:7]
        else:
            t0, t1 = times[idx-1], times[idx]
            ratio = (t - t0)/(t1 - t0)
            vx = self.imu_data[idx-1,1] + ratio*(self.imu_data[idx,1]-self.imu_data[idx-1,1])
            vy = self.imu_data[idx-1,2] + ratio*(self.imu_data[idx,2]-self.imu_data[idx-1,2])
            vz = self.imu_data[idx-1,3] + ratio*(self.imu_data[idx,3]-self.imu_data[idx-1,3])
            wx = self.imu_data[idx-1,4] + ratio*(self.imu_data[idx,4]-self.imu_data[idx-1,4])
            wy = self.imu_data[idx-1,5] + ratio*(self.imu_data[idx,5]-self.imu_data[idx-1,5])
            wz = self.imu_data[idx-1,6] + ratio*(self.imu_data[idx,6]-self.imu_data[idx-1,6])
        return np.array([vx,vy,vz]), np.array([wx,wy,wz])

    def interpolate_drone_pose(self, t):
        times = self.drone_times
        idx = np.searchsorted(times, t)
        if idx == 0:
            x,y,z = self.drone_x[0], self.drone_y[0], self.drone_z[0]
            wRi = self.wRi_list[0]
        elif idx >= len(times):
            x,y,z = self.drone_x[-1], self.drone_y[-1], self.drone_z[-1]
            wRi = self.wRi_list[-1]
        else:
            t0, t1 = times[idx-1], times[idx]
            ratio = (t - t0)/(t1 - t0)
            x = self.drone_x[idx-1] + ratio*(self.drone_x[idx]-self.drone_x[idx-1])
            y = self.drone_y[idx-1] + ratio*(self.drone_y[idx]-self.drone_y[idx-1])
            z = self.drone_z[idx-1] + ratio*(self.drone_z[idx]-self.drone_z[idx-1])
            # if ratio < 0.5:
            #     wRi = self.wRi_list[idx-1]
            # else:
            wRi = self.wRi_list[idx]

        return x,y,z,wRi

    def uwbCallback(self, msg):
        uwb_time = msg.system_time / 1e3
        if not self.uwb_init:
            self.uwb_init_time = uwb_time
            self.uwb_init = True

        t_rel = uwb_time - self.uwb_init_time
        currTime = t_rel

        if self.prevTime_ is not None:
            dt = currTime - self.prevTime_
        else:
            dt = 0.01

        if dt > 0 and len(msg.nodes) > 0:
            front_dists = np.zeros(4)
            back_dists = np.zeros(4)

            front_found = False
            back_found = False
            for node in msg.nodes:
                if node.id == 1:
                    front_dists[:] = [node.dis_arr[1], node.dis_arr[2], node.dis_arr[3], node.dis_arr[4]]
                    front_found = True
                elif node.id == 2:
                    back_dists[:] = [node.dis_arr[1], node.dis_arr[2], node.dis_arr[3], node.dis_arr[4]]
                    back_found = True

            if not front_found or not back_found:
                return

            z = np.zeros(8)
            z[:4] = front_dists
            z[4:] = back_dists

            vi, wi = self.interpolate_imu(t_rel)
            droneZ = self.interpolate_drone_pose(t_rel)[2]  
            self.ekf_.setHeight(droneZ)
            self.ekf_.setViWi(vi, wi)
            self.ekf_.setZ(z)
            self.ekf_.setDt(dt)
            self.ekf_.prediction()
            self.ekf_.correction()

            xji = self.ekf_.state["xji"]
            yji = self.ekf_.state["yji"]

            wx, wy, wz, wRi = self.interpolate_drone_pose(t_rel)

            iP_j = np.array([xji, yji, 0.22])
            wP_j = np.array([wx, wy, wz]) + wRi @ iP_j
            self.ekf_.pub_(wP_j)
            self.prevTime_ = currTime

    def read_gt(self):
        with open("hunter.txt", 'r') as file:
            for line in file:
                data = line.strip().split("\t")
                a, b, _, _ = map(float, data)
                self.hunter_x.append(a * 0.001) 
                self.hunter_y.append(b * 0.001)
        return

    def plot_data(self):
        while not rospy.is_shutdown():
            self.ax.clear()
            self.ax.scatter(self.hunter_x, self.hunter_y, c='r', label='Ground Truth', s=10)
            if self.ekf_x and self.ekf_y:
                self.ax.plot(self.ekf_x, self.ekf_y, c='b', label='EKF Path', linewidth=1)
            self.ax.legend()
            plt.draw()
            plt.pause(0.1) 
            rospy.sleep(0.1) 
if __name__ == "__main__":
    node = EKFNode()
    try:
        node.plot_data()
    except rospy.ROSInterruptException:
        pass
