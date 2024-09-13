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
        self.matR = 0.01 * np.eye(16, dtype=np.float32)
        self.mvecH = np.zeros(16, dtype=np.float32)       
        self.mvecZ = np.zeros(16, dtype=np.float32)       
        self.wRa = np.eye(3)
        rospy.Subscriber("/ranges", UwbRange, self.uwbCallback)
        rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.egoPoseCallback)
        rospy.Subscriber("/uav0/mavros/local_position/velocity_local", TwistStamped, self.velocityCallback)
        rospy.Subscriber("/uav0/mavros/imu/data", Imu, self.imuCallback)
        self.pose_pub = rospy.Publisher("/relative_pose", PoseStamped, queue_size=10)
        self.egoPosition = np.zeros(3)
        self.linearVelocity = np.zeros(3)
        self.angularVelocity = np.zeros(3)
        self.delta_t = 0
        self.orientation = None
        self.before_t = rospy.Time.now()
        # self.uwb_positions_drone1 = np.array([
        #     [0.17, 0.17, 0],    
        #     [0.17, -0.17, 0],   
        #     [-0.17, 0.17, 0],   
        #     [-0.17, -0.17, 0]
        # ]).T
        self.uwb_positions_drone1 = np.array([
            [0.5, 0.5, 0],    
            [0.5, -0.5, 0],   
            [-0.5, 0.5, 0],   
            [-0.5, -0.5, 0]
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
            self.linearVelocity = 0 

    def imuCallback(self, msg):
        self.angularVelocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def world_to_body_velocity(self, world_velocity):
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        self.wRa = tf.transformations.quaternion_matrix(quaternion)[:3, :3] 
        body_velocity = np.dot(self.wRa.T, world_velocity) 
        return body_velocity

    def process_data(self):
        self.mvecZ = np.array(self.range)
        current_time = rospy.Time.now()
        self.delta_t = (current_time - self.before_t).to_sec()
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
    
    def right_jacobian(self, omega):
        angle = np.linalg.norm(omega)
        if angle < 1e-9:
            return np.eye(3)  

        axis = omega / angle
        skew_axis = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])

        return np.eye(3) + np.eye(3) - (1 - np.cos(angle)) / (angle**2) * skew_axis + (angle - np.sin(angle)) / (angle**3) * (skew_axis @ skew_axis)
    
    def vectorToSkewSymmetric(self, vec):
        return np.array([[0, -vec[2], vec[1]],
                         [vec[2], 0, -vec[0]],
                         [-vec[1], vec[0], 0]], dtype=np.float32)

    def motionModel(self):
        R = self.state["aRb"]
        self.state["aPb"] += (self.vectorToSkewSymmetric(self.angularVelocity) @ self.state["aPb"]
                            - self.linearVelocity + R @ self.state["Vb"]) * self.delta_t
        self.state["aRb"] = R @ self.exp_map((R @ self.state["Wb"] - self.angularVelocity) * self.delta_t)
    
    def motionModelJacobian(self):
        R, Vb, Wb = self.state["aRb"], self.state["Vb"], self.state["Wb"]
        r = (R @ Wb - self.angularVelocity) * self.delta_t
        Fx = np.eye(12)
        Fx[0:3, 0:3] = np.eye(3) + self.vectorToSkewSymmetric(self.angularVelocity)*self.delta_t
        Fx[0:3, 3:6] = -np.dot(R, self.vectorToSkewSymmetric(Vb*self.delta_t))
        Fx[0:3, 6:9] = R * self.delta_t

        Fx[3:6, 3:6] = -self.right_jacobian(r)@ R @ self.vectorToSkewSymmetric(Wb*self.delta_t)
        # Fx[3:6, 9:12] = (R.T @ self.vectorToSkewSymmetric(Wb * self.delta_t).T @ self.exp_map(r) + R @ self.right_jacobian(r)) * R * self.delta_t
        Fx[3:6, 9:12] = self.right_jacobian(r) * R * self.delta_t

        self.mJacobianMatF = Fx
        #

    # def motionModelJacobian(self):
    #     R, Vb, Wb = self.state["aRb"], self.state["Vb"], self.state["Wb"]
    #     r = (R @ Wb - self.angularVelocity) * self.delta_t

    #     Fx = np.eye(12)
    #     Fx[0:3, 0:3] = np.eye(3) + self.vectorToSkewSymmetric(self.angularVelocity).T * self.delta_t
    #     Fx[0:3, 3:6] = -np.dot(R, self.vectorToSkewSymmetric(Vb * self.delta_t))
    #     Fx[0:3, 6:9] = R * self.delta_t

    #     mag_Wb = np.linalg.norm(Wb)
    #     if mag_Wb > 1e-9:
    #         skew_Wb = self.vectorToSkewSymmetric(Wb)
    #         Fx[3:6, 3:6] = self.exp_map(r).T - R @ self.right_jacobian(r) @ R @ skew_Wb
    #         Fx[3:6, 9:12] = (-R.T @ skew_Wb @ self.delta_t @ self.exp_map(r) + R @ self.right_jacobian(r)) * R * self.delta_t
    #     else:
    #         Fx[3:6, 3:6] = np.eye(3)
    #         Fx[3:6, 9:12] = np.zeros((3, 3))

    #     self.mJacobianMatF = Fx

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
                    H[idx, 3:6] = np.dot(diff.T / dist, self.vectorToSkewSymmetric(np.dot(self.state["aRb"],self.state["aPb"])))
                
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

        global_position = np.dot(self.wRa, self.state["aPb"]) + self.egoPosition

        relative_pose_msg.pose.position.x = global_position[0]
        relative_pose_msg.pose.position.y = global_position[1]
        relative_pose_msg.pose.position.z = global_position[2]

        global_rotation_matrix = np.dot(self.wRa, self.state["aRb"])
        quaternion = tf.transformations.quaternion_from_matrix(np.vstack([np.hstack([global_rotation_matrix, np.array([[0], [0], [0]])]), np.array([0, 0, 0, 1])]))
        relative_pose_msg.pose.orientation.x = quaternion[0]
        relative_pose_msg.pose.orientation.y = quaternion[1]
        relative_pose_msg.pose.orientation.z = quaternion[2]
        relative_pose_msg.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(relative_pose_msg)
if __name__ == "__main__":
    rospy.init_node("RelativePoseEstimation")
    Rel = RelativePoseEstimation()
    rospy.spin()
