# # import numpy as np
# # import rospy 
# # from relative.msg import UwbRange
# # from geometry_msgs.msg import PoseStamped
# # from sensor_msgs.msg import Imu
#sssss
# # class RelativePoseEstimation():
# #     def __init__ (self):
# #         self.state = {
# #             "aPb": np.zeros(3),
# #             "aRb": np.eye(3),
# #             "Vb": np.array(3),
# #             "Wb": np.array(3)
# #         } 
# #         self.matP = np.zeros((18,18), dtype=np.float32)
# #         self.matQ = np.zeros((18,18), dtype=np.float32)
# #         self.matR = np.zeros((16,16), dtype=np.float32)
# #         self.mvecH = np.zeros(16, dtype=np.float32)
# #         rospy.Subscriber("/uwb_node", UwbRange, self.uwbCallback)
# #         rospy.Subscriber("/result_ekf", PoseStamped, self.estimated)
# #         self.linearVelocity = []
# #         self.uwb_data_queue = []
# #         self.delta_t = 0
# #         self.before_t = None
# #         self.uwb_init = False
# #         self.imu_init = False
# #         self.range = UwbRange()
# #         self.egoPosition = np.zeros(3)

# #         self.tfB1 = np.array([[1,0,0,0],
# #                         [0,1,0,0],
# #                         [0,0,1,0],
# #                         [0,0,0,1]])
        
# #         self.tfB2 = np.array([[1,0,0,0],
# #                 [0,1,0,0],
# #                 [0,0,1,0],
# #                 [0,0,0,1]])
        
# #         self.tfB3 = np.array([[1,0,0,0],
# #                 [0,1,0,0],
# #                 [0,0,1,0],
# #                 [0,0,0,1]])
        
# #         self.tfB4 = np.array([[1,0,0,0],
# #                 [0,1,0,0],
# #                 [0,0,1,0],
# #                 [0,0,0,1]])

# #         self.tfA1 = np.array([[1,0,0,0],
# #                         [0,1,0,0],
# #                         [0,0,1,0],
# #                         [0,0,0,1]])
        
# #         self.tfA2 = np.array([[1,0,0,0],
# #                 [0,1,0,0],
# #                 [0,0,1,0],
# #                 [0,0,0,1]])
        
# #         self.tfA3 = np.array([[1,0,0,0],
# #                 [0,1,0,0],
# #                 [0,0,1,0],
# #                 [0,0,0,1]])
        
# #         self.tfA4 = np.array([[1,0,0,0],
# #                 [0,1,0,0],
# #                 [0,0,1,0],
# #                 [0,0,0,1]])
            
# #     def uwbCallback(self, msg):
# #         self.range = msg.ranges
# #         uwb_time = msg.system_time / 1e3  
# #         if not self.uwb_init:
# #             self.uwb_init_time = uwb_time
# #             self.uwb_init = True
# #         uwb_data = {
# #             "timestamp": uwb_time - self.uwb_init_time,
# #             "pos_3d": [msg.pos_3d[0], msg.pos_3d[1], msg.pos_3d[2]],
# #             "dis_arr": [msg.dis_arr[i] for i in range(8)]
# #         }
# #         self.uwb_data_queue.append(uwb_data)
# #         self.process_data()

# #     def estimated(self, msg):
# #         self.linearVelocity = msg.linearvelocity
# #         self.angularVelocity = msg.angularvelocity

# #     def process_data(self):
# #         if len(self.imu_data_queue) > 1 and len(self.uwb_data_queue) > 0:
# #             imu_data_1 = self.imu_data_queue[0]
# #             imu_data_2 = self.imu_data_queue[-1]
# #             uwb_data = self.uwb_data_queue[0]
# #             if imu_data_1["timestamp"] <= uwb_data["timestamp"] <= imu_data_2["timestamp"]:
# #                 t1 = imu_data_1["timestamp"]
# #                 t2 = imu_data_2["timestamp"]
# #                 t = uwb_data["timestamp"]

# #                 alpha = (t - t1) / (t2 - t1)

# #                 linear_acc = (1 - alpha) * np.array(imu_data_1["linear_acc"]) + alpha * np.array(imu_data_2["linear_acc"])
# #                 angular_vel = (1 - alpha) * np.array(imu_data_1["angular_vel"]) + alpha * np.array(imu_data_2["angular_vel"])

# #                 self.delta_t = t - self.before_t if self.before_t else 0
# #                 self.before_t = t
# #                 self.linear_acc = linear_acc
# #                 self.angular_vel = angular_vel

# #                 self.m_vecZ = np.array(uwb_data["dis_arr"])

# #                 self.prediction()
# #                 self.correction()

# #                 self.uwb_data_queue.clear()

# #     def exp_map(self, omega):
# #         angle = np.linalg.norm(omega)
# #         if angle < 1e-15:
# #             return np.eye(3)
# #         axis = omega / angle
# #         K = np.array([
# #             [0, -axis[2], axis[1]],
# #             [axis[2], 0, -axis[0]],
# #             [-axis[1], axis[0], 0]
# #         ])
# #         return np.cos(angle) * np.eye(3) + (1 - np.cos(angle)) * np.outer(axis, axis) + np.sin(angle) * K

# #     def vectorToSkewSymmetric(self, vec):
# #         return np.array([[0, -vec[2], vec[1]],
# #                          [vec[2], 0, -vec[0]],
# #                          [-vec[1], vec[0], 0]], dtype=np.float32)
        
# #     def motionModel(self):
# #         R = self.state["aRb"]
# #         self.state["aPb"] = self.state["aPb"] + (self.vectorToSkewSymmetric(self.angularVelocity)*self.state["aPb"] \
# #                             - self.linearVelocity + R * self.state["Vb"])*self.delta_t
# #         self.state["aRb"] = R*self.exp_map(self.vectorToSkewSymmetric(R*self.state["Wb"]-self.angularVelocity)*self.delta_t)
# #         self.state["Vb"] = self.state["Vb"]
# #         self.state["Wb"] = self.state["Wb"]

# #     def motionModelJacobian(self):
# #         P = self.state["aPb"]
# #         R = self.state["aRb"]
# #         Vb = self.state["Vb"]
# #         Wb = self.state["Wb"]
# #         r = (R*Wb - self.angularVelocity)*self.delta_t
# #         mag_r = np.linalg.norm(r)
# #         skew_r = self.vectorToSkewSymmetric(r/mag_r)
# #         Fx = np.eye(12)
# #         Fx[0:3,3:6] = Vb*self.delta_t
# #         Fx[0:3,6:9] = R*self.delta_t

# #         Fx[3:6,3:6] = self.exp_map(r) + R*((np.eye(3)-(1-np.cos(mag_r))/(mag_r**2)*skew_r+(mag_r-np.sin(mag_r))/(mag_r**3)*skew_r**2)*Wb*self.delta_t)
# #         Fx[3:6,9:12] = R*self.exp_map(r)*self.delta_t
# #         self.mJacobianMatF = Fx

# #     def prediction(self):
# #         self.motionModelJacobian()
# #         self.motionModel()
# #         self.matP = np.dot(np.dot(self.mJacobianMatF, self.matP), self.mJacobianMatF.T) + self.matQ

# #     def measurementModel(self, vec):
# #         self.mvecH[0] = np.linalg.norm(vec[:3]*self.tfB1 - self.egoPosition*self.tfA1)
# #         self.mvecH[1] = np.linalg.norm(vec[:3]*self.tfB1 - self.egoPosition*self.tfA2)
# #         self.mvecH[2] = np.linalg.norm(vec[:3]*self.tfB1 - self.egoPosition*self.tfA3)
# #         self.mvecH[3] = np.linalg.norm(vec[:3]*self.tfB1 - self.egoPosition*self.tfA4)
# #         self.mvecH[4] = np.linalg.norm(vec[:3]*self.tfB2 - self.egoPosition*self.tfA1)
# #         self.mvecH[5] = np.linalg.norm(vec[:3]*self.tfB2 - self.egoPosition*self.tfA2)
# #         self.mvecH[6] = np.linalg.norm(vec[:3]*self.tfB2 - self.egoPosition*self.tfA3)
# #         self.mvecH[7] = np.linalg.norm(vec[:3]*self.tfB2 - self.egoPosition*self.tfA4)
# #         self.mvecH[8] = np.linalg.norm(vec[:3]*self.tfB3 - self.egoPosition*self.tfA1)
# #         self.mvecH[9] = np.linalg.norm(vec[:3]*self.tfB3 - self.egoPosition*self.tfA2)
# #         self.mvecH[10] = np.linalg.norm(vec[:3]*self.tfB3 - self.egoPosition*self.tfA3)
# #         self.mvecH[11] = np.linalg.norm(vec[:3]*self.tfB3 - self.egoPosition*self.tfA4)
# #         self.mvecH[12] = np.linalg.norm(vec[:3]*self.tfB4 - self.egoPosition*self.tfA1)
# #         self.mvecH[13] = np.linalg.norm(vec[:3]*self.tfB4 - self.egoPosition*self.tfA2)
# #         self.mvecH[14] = np.linalg.norm(vec[:3]*self.tfB4 - self.egoPosition*self.tfA3)
# #         self.mvecH[15] = np.linalg.norm(vec[:3]*self.tfB4 - self.egoPosition*self.tfA4)        

# #     def measurementModelJacobian(self, vec):
# #         H = np.zeros((16, 12), dtype=np.float32)
# #         self.mJacobianMatH = (vec[:3]*self.tfB1 - self.egoPosition*self.tfA1)[0]/self.mvecH[0]
# #         self.mJacobianMatH = (vec[:3]*self.tfB1 - self.egoPosition*self.tfA1)[1]/self.mvecH[0]
# #         self.mJacobianMatH = (vec[:3]*self.tfB1 - self.egoPosition*self.tfA1)[2]/self.mvecH[0]
# #         ...

# #     def correction(self):
# #         stateVec = self.state["aPb"]
# #         self.measurementModel(stateVec)
# #         self.measurementModelJacobian(stateVec)

# # if __name__=="__main__":
# #     rospy.init_node("RelativePoseEstimation")   
# #     Rel = RelativePoseEstimation()    
# #     rospy.spin()

# import numpy as np
# import rospy
# from relative.msg import UwbRange
# from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import Imu

# class RelativePoseEstimation():
#     def __init__ (self):
#         self.state = {
#             "aPb": np.zeros(3),
#             "aRb": np.eye(3),
#             "Vb": np.zeros(3),
#             "Wb": np.zeros(3)
#         }
#         self.matP = np.zeros((12, 12), dtype=np.float32)
#         self.matQ = np.zeros((12, 12), dtype=np.float32)
#         self.matR = np.zeros((16, 16), dtype=np.float32)
#         self.mvecH = np.zeros(16, dtype=np.float32)
#         self.mvecZ = np.zeros(16, dtype=np.float32)
#         rospy.Subscriber("/uwb_node", UwbRange, self.uwbCallback)
#         rospy.Subscriber("/result_ekf", PoseStamped, self.estimated)
#         self.linearVelocity = []
#         self.uwb_data_queue = []
#         self.delta_t = 0
#         self.before_t = None
#         self.uwb_init = False
#         self.range = UwbRange()
#         self.egoPosition = np.zeros(3)

#         self.tfB = [np.eye(4) for _ in range(4)]
#         self.tfA = [np.eye(4) for _ in range(4)]
        
#     def uwbCallback(self, msg):
#         self.range = msg.ranges
#         self.process_data()

#     def estimated(self, msg):
#         self.egoPosition = msg.pose.position
#         self.linearVelocity = msg.linearvelocity
#         self.angularVelocity = msg.angularvelocity

#     def process_data(self):
#         self.mvecZ = np.array(self.range)
#         self.prediction()
#         self.correction()

#     def exp_map(self, omega):
#         angle = np.linalg.norm(omega)
#         if angle < 1e-15:
#             return np.eye(3)
#         axis = omega / angle
#         K = np.array([
#             [0, -axis[2], axis[1]],
#             [axis[2], 0, -axis[0]],
#             [-axis[1], axis[0], 0]
#         ])
#         return np.cos(angle) * np.eye(3) + (1 - np.cos(angle)) * np.outer(axis, axis) + np.sin(angle) * K

#     def vectorToSkewSymmetric(self, vec):
#         return np.array([[0, -vec[2], vec[1]],
#                          [vec[2], 0, -vec[0]],
#                          [-vec[1], vec[0], 0]], dtype=np.float32)

#     def motionModel(self):
#         R = self.state["aRb"]
#         self.state["aPb"] += (self.vectorToSkewSymmetric(self.angularVelocity) @ self.state["aPb"]
#                             - self.linearVelocity + R @ self.state["Vb"]) * self.delta_t
#         self.state["aRb"] = R @ self.exp_map(self.vectorToSkewSymmetric(R @ self.state["Wb"] - self.angularVelocity) * self.delta_t)

#     def motionModelJacobian(self):
#         R, Vb, Wb = self.state["aRb"], self.state["Vb"], self.state["Wb"]
#         r = (R @ Wb - self.angularVelocity) * self.delta_t
#         mag_r = np.linalg.norm(r)
#         skew_r = self.vectorToSkewSymmetric(r / mag_r)
#         Fx = np.eye(12)
#         Fx[0:3, 3:6] = Vb * self.delta_t
#         Fx[0:3, 6:9] = R * self.delta_t
#         Fx[3:6, 3:6] = self.exp_map(r) + R @ ((np.eye(3) - (1 - np.cos(mag_r)) / (mag_r**2) * skew_r
#                           + (mag_r - np.sin(mag_r)) / (mag_r**3) * skew_r**2) * Wb * self.delta_t)
#         Fx[3:6, 9:12] = R @ self.exp_map(r) * self.delta_t
#         self.mJacobianMatF = Fx

#     def prediction(self):
#         self.motionModelJacobian()
#         self.motionModel()
#         self.matP = self.mJacobianMatF @ self.matP @ self.mJacobianMatF.T + self.matQ

#     def measurementModel(self, vec):
#         for i in range(4):
#             for j in range(4):
#                 idx = i * 4 + j
#                 self.mvecH[idx] = np.linalg.norm(vec[:3] @ self.tfB[i] - self.egoPosition @ self.tfA[j])

#     def measurementModelJacobian(self, vec):
#         H = np.zeros((16, 12), dtype=np.float32)

#         for i in range(4):
#             for j in range(4):
#                 idx = i * 4 + j
#                 diff = vec[:3] @ self.tfB[i][:3, :3] - self.egoPosition @ self.tfA[j][:3, :3]
#                 norm_diff = np.linalg.norm(diff)

#                 if norm_diff > 1e-6: 
#                     H[idx, :3] = diff / norm_diff  
#                 else:
#                     H[idx, :3] = np.zeros(3)

#         self.mJacobianMatH = H

#     def correction(self):
#         stateVec = self.state["aPb"]
#         self.measurementModel(stateVec)
#         self.measurementModelJacobian(stateVec)
#         residual = self.mvecZ - self.mvecH
#         residualCov = np.dot(np.dot(self.mJacobianMatH, self.matP), self.mJacobianMatH.T) + self.matR
#         Kk = np.dot(np.dot(self.matP, self.mJacobianMatH.T), np.linalg.inv(residualCov))
#         stateUpdate = np.dot(Kk, residual)
#         self.state["aPb"] += stateUpdate[:3]
#         self.state["aRb"] = self.state["aRb"]*self.exp_map(stateUpdate[3:6])
#         self.state["Vb"] += stateUpdate[6:9] 
#         self.state["Wb"] += stateUpdate[9:12]
#         self.matP = np.dot((np.eye(12) - np.dot(Kk, self.mJacobianMatH)), self.matP)

# if __name__ == "__main__":
#     rospy.init_node("RelativePoseEstimation")
#     Rel = RelativePoseEstimation()
#     rospy.spin()
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
        self.matP = np.zeros((12, 12), dtype=np.float32)  
        self.matQ = 0.001 * np.eye(12, dtype=np.float32)
        self.matR = 0.001 * np.eye(16, dtype=np.float32)
        self.mvecH = np.zeros(16, dtype=np.float32)       
        self.mvecZ = np.zeros(16, dtype=np.float32)       
        rospy.Subscriber("/ranges", UwbRange, self.uwbCallback)
        rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.egoPoseCallback)
        rospy.Subscriber("/uav0/mavros/local_position/velocity_local", TwistStamped, self.velocityCallback)
        rospy.Subscriber("/uav0/mavros/imu/data", Imu, self.imuCallback)
        self.linearVelocity = np.zeros(3)
        self.angularVelocity = np.zeros(3)
        self.uwb_data = None
        self.before_t = rospy.Time.now()
        self.uwb_positions_drone1 = np.array([
            [0.17, 0.17, 0],    
            [0.17, -0.17, 0],   
            [-0.17, 0.17, 0],   
            [-0.17, -0.17, 0]
        ]).T
        self.uwb_positions_drone2 = self.uwb_positions_drone1

    def uwbCallback(self, msg):
        self.range = msg.ranges
        self.process_data()

    def egoPoseCallback(self, msg):
        self.egoPosition = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.orientation = msg.pose.orientation 

    def velocityCallback(self, msg):
        world_linear_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        if self.orientation is not None:
            self.linearVelocity = self.world_to_body_velocity(world_linear_velocity)
        else:
            self.linearVelocity = world_linear_velocity 

    def imuCallback(self, msg):
        self.angularVelocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def world_to_body_velocity(self, world_velocity):
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3] 
        body_velocity = np.dot(rotation_matrix.T, world_velocity) 
        return body_velocity

    def process_data(self):
        self.mvecZ = np.array(self.range)
        current_time = rospy.Time.now()
        self.uwb_data = (current_time - self.before_t).to_sec()
        self.before_t = current_time
        self.prediction()
        self.correction()

    def exp_map(self, omega):
        angle = np.linalg.norm(omega)
        if angle < 1e-9:
            return np.eye(3)
        axis = omega / angle
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        return np.cos(angle) * np.eye(3) + (1 - np.cos(angle)) * np.outer(axis, axis) + np.sin(angle) * K

    def vectorToSkewSymmetric(self, vec):
        return np.array([[0, -vec[2], vec[1]],
                         [vec[2], 0, -vec[0]],
                         [-vec[1], vec[0], 0]], dtype=np.float32)

    def motionModel(self):
        R = self.state["aRb"]
        self.state["aPb"] += (self.vectorToSkewSymmetric(self.angularVelocity) @ self.state["aPb"]
                            - self.linearVelocity + R @ self.state["Vb"]) * self.uwb_data
        self.state["aRb"] = R @ self.exp_map((R @ self.state["Wb"] - self.angularVelocity) * self.uwb_data)

    def motionModelJacobian(self):
        R, Vb, Wb = self.state["aRb"], self.state["Vb"], self.state["Wb"]
        r = (R @ Wb - self.angularVelocity) * self.uwb_data
        mag_r = np.linalg.norm(r)
        skew_r = self.vectorToSkewSymmetric(r / mag_r)
        Fx = np.eye(12)
        Fx[0:3, 3:6] = Vb * self.uwb_data
        Fx[0:3, 6:9] = R * self.uwb_data
        Fx[3:6, 3:6] = self.exp_map(r) + R @ ((np.eye(3) - (1 - np.cos(mag_r)) / (mag_r**2) * skew_r
                          + (mag_r - np.sin(mag_r)) / (mag_r**3) * skew_r**2) * Wb * self.uwb_data)
        Fx[3:6, 9:12] = R @ self.exp_map(r) * self.uwb_data
        self.mJacobianMatF = Fx

    def prediction(self):
        self.motionModelJacobian()
        self.motionModel()
        self.matP = self.mJacobianMatF @ self.matP @ self.mJacobianMatF.T + self.matQ

    def measurementModel(self):
        idx = 0
        for i in range(4):
            for j in range(4):
                sensor_1_position = self.state["aRb"] @ self.uwb_positions_drone1[:, i] + self.state["aPb"]
                sensor_2_position = self.uwb_positions_drone2[:, j]
                self.mvecH[idx] = np.linalg.norm(sensor_1_position - sensor_2_position)
                idx += 1

    def measurementModelJacobian(self):
        H = np.zeros((16, 12), dtype=np.float32)
        idx = 0
        for i in range(4):
            for j in range(4):
                sensor_1_position = self.state["aRb"] @ self.uwb_positions_drone1[:, i] + self.state["aPb"]
                sensor_2_position = self.uwb_positions_drone2[:, j]
                diff = sensor_1_position - sensor_2_position
                dist = np.linalg.norm(diff)

                if dist > 1e-6:
                    H[idx, :3] = diff / dist
                    H[idx, 3:6] = np.cross(self.uwb_positions_drone1[:, i], diff) / dist
                
                idx += 1

        self.mJacobianMatH = H

    def correction(self):
        self.measurementModel()
        self.measurementModelJacobian()
        residual = self.mvecZ - self.mvecH
        residualCov = np.dot(np.dot(self.mJacobianMatH, self.matP), self.mJacobianMatH.T) + self.matR
        Kk = np.dot(np.dot(self.matP, self.mJacobianMatH.T), np.linalg.inv(residualCov))
        stateUpdate = np.dot(Kk, residual)
        self.state["aPb"] += stateUpdate[:3]
        self.state["aRb"] = self.state["aRb"] @ self.exp_map(stateUpdate[3:6])
        self.state["Vb"] += stateUpdate[6:9]
        self.state["Wb"] += stateUpdate[9:12]
        self.matP = np.dot((np.eye(12) - np.dot(Kk, self.mJacobianMatH)), self.matP)

if __name__ == "__main__":
    rospy.init_node("RelativePoseEstimation")
    Rel = RelativePoseEstimation()
    rospy.spin()

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
        self.matP = np.zeros((12, 12), dtype=np.float32)  
        self.matQ = 0.001 * np.eye(12, dtype=np.float32)
        self.matR = 0.001 * np.eye(16, dtype=np.float32)
        self.mvecH = np.zeros(16, dtype=np.float32)       
        self.mvecZ = np.zeros(16, dtype=np.float32)       
        rospy.Subscriber("/ranges", UwbRange, self.uwbCallback)
        rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.egoPoseCallback)
        rospy.Subscriber("/uav0/mavros/local_position/velocity_local", TwistStamped, self.velocityCallback)
        rospy.Subscriber("/uav0/mavros/imu/data", Imu, self.imuCallback)
        self.pose_pub = rospy.Publisher("/relative_pose", PoseStamped, queue_size=10)
        
        self.linearVelocity = np.zeros(3)
        self.angularVelocity = np.zeros(3)
        self.uwb_data = None
        self.before_t = rospy.Time.now()
        self.uwb_positions_drone1 = np.array([
            [0.17, 0.17, 0],    
            [0.17, -0.17, 0],   
            [-0.17, 0.17, 0],   
            [-0.17, -0.17, 0]
        ]).T
        self.uwb_positions_drone2 = self.uwb_positions_drone1

    def uwbCallback(self, msg):
        self.range = msg.ranges
        self.process_data()

    def egoPoseCallback(self, msg):
        self.egoPosition = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.orientation = msg.pose.orientation 

    def velocityCallback(self, msg):
        world_linear_velocity = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        if self.orientation is not None:
            self.linearVelocity = self.world_to_body_velocity(world_linear_velocity)
        else:
            self.linearVelocity = world_linear_velocity 

    def imuCallback(self, msg):
        self.angularVelocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def world_to_body_velocity(self, world_velocity):
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3] 
        body_velocity = np.dot(rotation_matrix.T, world_velocity) 
        return body_velocity

    def process_data(self):
        self.mvecZ = np.array(self.range)
        current_time = rospy.Time.now()
        self.uwb_data = (current_time - self.before_t).to_sec()
        self.before_t = current_time
        self.prediction()
        self.correction()
        self.publish_pose()

    def exp_map(self, omega):
        angle = np.linalg.norm(omega)
        if angle < 1e-9:
            return np.eye(3)
        axis = omega / angle
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        return np.cos(angle) * np.eye(3) + (1 - np.cos(angle)) * np.outer(axis, axis) + np.sin(angle) * K

    def vectorToSkewSymmetric(self, vec):
        return np.array([[0, -vec[2], vec[1]],
                         [vec[2], 0, -vec[0]],
                         [-vec[1], vec[0], 0]], dtype=np.float32)

    def motionModel(self):
        R = self.state["aRb"]
        self.state["aPb"] += (self.vectorToSkewSymmetric(self.angularVelocity) @ self.state["aPb"]
                            - self.linearVelocity + R @ self.state["Vb"]) * self.uwb_data
        self.state["aRb"] = R @ self.exp_map((R @ self.state["Wb"] - self.angularVelocity) * self.uwb_data)

    def motionModelJacobian(self):
        R, Vb, Wb = self.state["aRb"], self.state["Vb"], self.state["Wb"]
        r = (R @ Wb - self.angularVelocity) * self.uwb_data
        mag_r = np.linalg.norm(r)
        skew_r = self.vectorToSkewSymmetric(r / mag_r)
        Fx = np.eye(12)
        Fx[0:3, 3:6] = Vb * self.uwb_data
        Fx[0:3, 6:9] = R * self.uwb_data
        Fx[3:6, 3:6] = self.exp_map(r) + R @ ((np.eye(3) - (1 - np.cos(mag_r)) / (mag_r**2) * skew_r
                          + (mag_r - np.sin(mag_r)) / (mag_r**3) * skew_r**2) * Wb * self.uwb_data)
        Fx[3:6, 9:12] = R @ self.exp_map(r) * self.uwb_data
        self.mJacobianMatF = Fx

    def prediction(self):
        self.motionModelJacobian()
        self.motionModel()
        self.matP = self.mJacobianMatF @ self.matP @ self.mJacobianMatF.T + self.matQ

    def measurementModel(self):
        idx = 0
        for i in range(4):
            for j in range(4):
                sensor_1_position = self.state["aRb"] @ self.uwb_positions_drone1[:, i] + self.state["aPb"]
                sensor_2_position = self.uwb_positions_drone2[:, j]
                self.mvecH[idx] = np.linalg.norm(sensor_1_position - sensor_2_position)
                idx += 1

    def measurementModelJacobian(self):
        H = np.zeros((16, 12), dtype=np.float32)
        idx = 0
        for i in range(4):
            for j in range(4):
                sensor_1_position = self.state["aRb"] @ self.uwb_positions_drone1[:, i] + self.state["aPb"]
                sensor_2_position = self.uwb_positions_drone2[:, j]
                diff = sensor_1_position - sensor_2_position
                dist = np.linalg.norm(diff)

                if dist > 1e-6:
                    H[idx, :3] = diff / dist
                    H[idx, 3:6] = np.cross(self.uwb_positions_drone1[:, i], diff) / dist
                
                idx += 1

        self.mJacobianMatH = H

    def correction(self):
        self.measurementModel()
        self.measurementModelJacobian()
        residual = self.mvecZ - self.mvecH
        residualCov = np.dot(np.dot(self.mJacobianMatH, self.matP), self.mJacobianMatH.T) + self.matR
        Kk = np.dot(np.dot(self.matP, self.mJacobianMatH.T), np.linalg.inv(residualCov))
        stateUpdate = np.dot(Kk, residual)
        self.state["aPb"] += stateUpdate[:3]
        self.state["aRb"] = self.state["aRb"] @ self.exp_map(stateUpdate[3:6])
        self.state["Vb"] += stateUpdate[6:9]
        self.state["Wb"] += stateUpdate[9:12]
        self.matP = np.dot((np.eye(12) - np.dot(Kk, self.mJacobianMatH)), self.matP)

    def publish_pose(self):
        relative_pose_msg = PoseStamped()
        relative_pose_msg.header.stamp = rospy.Time.now()
        relative_pose_msg.pose.position.x = self.state["aPb"][0]
        relative_pose_msg.pose.position.y = self.state["aPb"][1]
        relative_pose_msg.pose.position.z = self.state["aPb"][2]

        self.pose_pub.publish(relative_pose_msg)
if __name__ == "__main__":
    rospy.init_node("RelativePoseEstimation")
    Rel = RelativePoseEstimation()
    rospy.spin()
