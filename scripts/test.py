import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
import numpy as np
from relative.msg import UwbRange

class RelativePoseEstimation():
    def __init__(self):
        self.state = {
            "aPb": np.zeros(3),       
            "aRb": np.eye(3),        
            "Vb": np.zeros(3),        
            "Wb": np.zeros(3)         
        }
        self.matP = np.eye(12) * 0.1  
        self.matQ = 0.001 * np.eye(12)  
        self.matR = 0.01 * np.eye(16)   
        self.mvecH = np.zeros(16)      
        self.mvecZ = np.zeros(16)      
        self.wRa = np.eye(3)          
        self.aVw = np.zeros(3)         
        self.aWw = np.zeros(3)          
        self.aVa = np.zeros(3)  
        rospy.Subscriber("/ranges", UwbRange, self.uwbCallback)
        rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.egoPoseCallback)
        rospy.Subscriber("/uav0/mavros/local_position/velocity_body", TwistStamped, self.velocityCallback)
        rospy.Subscriber("/uav0/mavros/imu/data", Imu, self.imuCallback)

        self.pose_pub = rospy.Publisher("/relative_pose", PoseStamped, queue_size=10)

        self.egoPosition = np.zeros(3)
        self.egoOrientation = np.array([0, 0, 0, 1])  #(x, y, z, w)
        self.linearVelocity = np.zeros(3)
        self.angularVelocity = np.zeros(3)
        self.delta_t = 0.01
        self.before_t = rospy.Time.now()

        self.uwb_positions_drone1 = np.array([
            [0.5, 0.5, 0],    
            [0.5, -0.5, 0],   
            [-0.5, 0.5, 0],   
            [-0.5, -0.5, 0]
        ]).T  

        self.uwb_positions_drone2 = self.uwb_positions_drone1.copy()

    def uwbCallback(self, msg):
        self.mvecZ = msg.ranges 
        self.process_data()


    def egoPoseCallback(self, msg):
        self.egoPosition = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.egoOrientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        self.wRa = tf.transformations.quaternion_matrix(self.egoOrientation)[:3, :3]  

    def velocityCallback(self, msg):
        self.aVa = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    def imuCallback(self, msg):
        self.angularVelocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def process_data(self):
        current_time = rospy.Time.now()
        self.delta_t = (current_time - self.before_t).to_sec()
        if self.delta_t <= 0:
            self.delta_t = 0.01 
        self.before_t = current_time

        self.prediction()
        self.correction()
        self.publish_pose()

    def exp_map(self, omega):
        angle = np.linalg.norm(omega)
        if angle < 1e-9:
            return np.eye(3)
        axis = omega / angle
        K = self.vectorToSkewSymmetric(axis)
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    def vectorToSkewSymmetric(self, vec):
        return np.array([[0, -vec[2], vec[1]],
                         [vec[2], 0, -vec[0]],
                         [-vec[1], vec[0], 0]])

    def motionModel(self):
        R = self.state["aRb"]          
        Vb = self.state["Vb"]            
        Wb = self.state["Wb"]           
        delta_t = self.delta_t

        aVb = R @ Vb                     
        aWb = R @ Wb                     

        aWa = self.angularVelocity      

        # aVa = self.wRa.T @ self.aVw      

        aPb_dot = self.vectorToSkewSymmetric(aWa) @ self.state["aPb"] - self.aVa + aVb
        self.state["aPb"] += aPb_dot * delta_t

        omega = (aWb - aWa)
        self.state["aRb"] = self.state["aRb"] @ self.exp_map(omega * delta_t)

    def motionModelJacobian(self):
        R = self.state["aRb"]
        Vb = self.state["Vb"]
        Wb = self.state["Wb"]
        delta_t = self.delta_t

        Fx = np.eye(12)

        aVb = R @ Vb
        aWb = R @ Wb

        aWa = self.angularVelocity

        # aVa = self.wRa.T @ self.aVw

        Fx[0:3, 0:3] += self.vectorToSkewSymmetric(aWa) * delta_t
        Fx[0:3, 3:6] += -R @ self.vectorToSkewSymmetric(Vb) * delta_t
        Fx[0:3, 6:9] += R * delta_t

        # Fx[3:6, 3:6] += -self.vectorToSkewSymmetric(aWa) * delta_t
        Fx[3:6, 9:12] += R * delta_t

        self.mJacobianMatF = Fx

    def prediction(self):
        self.motionModelJacobian()
        self.motionModel()
        self.matP = self.mJacobianMatF @ self.matP @ self.mJacobianMatF.T + self.matQ * self.delta_t

    def measurementModel(self):
        idx = 0
        for i in range(4):
            for j in range(4):
                # aPi = self.state["aRb"] @ self.uwb_positions_drone1[:, i] + self.state["aPb"]
                aPi = self.uwb_positions_drone1[:, i]
                aPj = self.state["aPb"] + self.state["aRb"] @ self.uwb_positions_drone2[:, j]
                diff = aPi - aPj
                # print(aPj)
                # print("diff:",diff)
                # print("diff_norm:",np.linalg.norm(diff))
                self.mvecH[idx] = np.linalg.norm(diff)
                idx += 1
        # print(self.mvecH)

    def measurementModelJacobian(self):
        H = np.zeros((16, 12))
        idx = 0
        for i in range(4):
            for j in range(4):
                # aPi = self.state["aRb"] @ self.uwb_positions_drone1[:, i] + self.state["aPb"]
                aPi = self.uwb_positions_drone1[:, i]
                aPj = self.state["aPb"] + self.state["aRb"] @ self.uwb_positions_drone2[:, j]
                diff = aPi - aPj
                dist = np.linalg.norm(diff)


                H[idx, 0:3] = -diff / dist 
                # print(H[idx, 0:3])
                skew_i = self.vectorToSkewSymmetric(self.uwb_positions_drone1[:, i])
                skew_j = self.vectorToSkewSymmetric(self.uwb_positions_drone2[:, j])
                pj_b = self.uwb_positions_drone2[:, j]
                H[idx, 3:6] = - ( (self.state["aRb"] @ self.vectorToSkewSymmetric(pj_b)) @ diff ) / dist

                idx += 1

        self.mJacobianMatH = H

    def correction(self):
        self.measurementModel()
        self.measurementModelJacobian()
        residual = self.mvecZ - self.mvecH
        print(residual)
        residualCov = self.mJacobianMatH @ self.matP @ self.mJacobianMatH.T + self.matR
        Kk = self.matP @ self.mJacobianMatH.T @ np.linalg.inv(residualCov)
        stateUpdate = Kk @ residual

        self.state["aPb"] += stateUpdate[0:3]
        # self.state["aPb"][2] = self.egoPosition[2]
        delta_rot = self.exp_map(stateUpdate[3:6])
        self.state["aRb"] = self.state["aRb"] @ delta_rot 
        self.state["Vb"] += stateUpdate[6:9]
        self.state["Wb"] += stateUpdate[9:12]

        self.matP = (np.eye(12) - Kk @ self.mJacobianMatH) @ self.matP

    def publish_pose(self):
        relative_pose_msg = PoseStamped()
        relative_pose_msg.header.stamp = rospy.Time.now()
        relative_pose_msg.header.frame_id = "map"

        # global_position = self.wRa @ self.state["aPb"] + self.egoPosition
        global_position = self.wRa @ self.state["aPb"] 
        relative_pose_msg.pose.position.x = global_position[0]
        relative_pose_msg.pose.position.y = global_position[1]
        relative_pose_msg.pose.position.z = global_position[2]

        global_rotation_matrix = self.wRa @ self.state["aRb"]
        quaternion = tf.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((global_rotation_matrix, np.array([[0], [0], [0]]))), np.array([0, 0, 0, 1])))
        )
        relative_pose_msg.pose.orientation.x = quaternion[0]
        relative_pose_msg.pose.orientation.y = quaternion[1]
        relative_pose_msg.pose.orientation.z = quaternion[2]
        relative_pose_msg.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(relative_pose_msg)

if __name__ == "__main__":
    rospy.init_node("RelativePoseEstimation")
    rel_pose_estimator = RelativePoseEstimation()
    rospy.spin()
