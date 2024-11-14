import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
import numpy as np
from relative.msg import UwbRange

class RelativePoseEstimation2D():
    def __init__(self):
        self.state = {
            "aPb": np.zeros(2),       # 드론 B의 위치 (프레임 A에서, 2D)
            "aRb": np.eye(2),         # 드론 B의 자세를 프레임 A로 변환하는 2D 회전 행렬
            "Vb": np.zeros(2),        # 드론 B의 속도 (2D)
            "Wb": 0.0                 # 드론 B의 각속도 (스칼라)
        }
        self.matP = np.eye(5) * 0.1  # 초기 공분산 행렬 (5x5)
        self.matQ = 0.001 * np.eye(5)  # 프로세스 노이즈 공분산
        self.matR = 0.01 * np.eye(16)   # 측정 노이즈 공분산
        self.mvecH = np.zeros(16)       # 예측된 측정값
        self.mvecZ = np.zeros(16)       # 실제 측정값
        self.wRa = np.eye(2)            # 프레임 A에서 월드 프레임으로의 회전 (2D)
        self.aVw = np.zeros(2)          # 드론 A의 속도 (월드 프레임에서, 2D)
        self.aWw = 0.0                  # 드론 A의 각속도 (스칼라)

        # ROS 구독자 및 발행자 설정
        rospy.Subscriber("/ranges", UwbRange, self.uwbCallback)
        rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.egoPoseCallback)
        rospy.Subscriber("/uav0/mavros/local_position/velocity_local", TwistStamped, self.velocityCallback)
        rospy.Subscriber("/uav0/mavros/imu/data", Imu, self.imuCallback)
        
        self.pose_pub = rospy.Publisher("/relative_pose", PoseStamped, queue_size=10)
        
        # 초기화
        self.egoPosition = np.zeros(2)
        self.egoOrientation = 0.0  # 드론 A의 Yaw 각도 (라디안)
        self.linearVelocity = np.zeros(2)
        self.angularVelocity = 0.0
        self.delta_t = 0.01
        self.before_t = rospy.Time.now()

        # 드론의 UWB 노드 위치 (2D)
        self.uwb_positions_drone1 = np.array([
            [0.5, 0.5],    
            [0.5, -0.5],   
            [-0.5, 0.5],   
            [-0.5, -0.5]
        ]).T  # Shape: (2, 4)

        self.uwb_positions_drone2 = self.uwb_positions_drone1.copy()

    def uwbCallback(self, msg):
        self.ranges = msg.ranges  # UwbRange 메시지의 ranges 필드 사용
        if len(self.ranges) == 16:
            self.process_data()
        else:
            rospy.logwarn("Received incorrect number of UWB ranges.")

    def egoPoseCallback(self, msg):
        self.egoPosition = np.array([msg.pose.position.x, msg.pose.position.y])
        # 쿼터니언에서 Yaw 각도 추출
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.egoOrientation = euler[2]  # Yaw 각도
        # 회전 행렬 계산
        self.wRa = np.array([
            [np.cos(self.egoOrientation), -np.sin(self.egoOrientation)],
            [np.sin(self.egoOrientation),  np.cos(self.egoOrientation)]
        ])

    def velocityCallback(self, msg):
        # 월드 프레임에서의 속도 (2D)
        self.aVw = np.array([msg.twist.linear.x, msg.twist.linear.y])

    def imuCallback(self, msg):
        # 각속도의 Z축만 사용 (Yaw)
        self.angularVelocity = msg.angular_velocity.z

    def process_data(self):
        current_time = rospy.Time.now()
        self.delta_t = (current_time - self.before_t).to_sec()
        if self.delta_t <= 0:
            self.delta_t = 0.01  # 기본 시간 간격
        self.before_t = current_time

        self.prediction()
        self.correction()
        self.publish_pose()

    def exp_map(self, omega):
        # 2D 회전 행렬 생성
        return np.array([
            [np.cos(omega), -np.sin(omega)],
            [np.sin(omega),  np.cos(omega)]
        ])

    def vectorToSkewSymmetric(self, omega):
        # 2D에서는 스큐 대칭 행렬이 스칼라에 대응
        return np.array([[0, -omega],
                         [omega, 0]])

    def motionModel(self):
        R = self.state["aRb"]            # 드론 B의 자세 (2D 회전 행렬)
        Vb = self.state["Vb"]            # 드론 B의 속도 (2D)
        Wb = self.state["Wb"]            # 드론 B의 각속도 (스칼라)
        delta_t = self.delta_t

        # 속도를 프레임 A로 변환
        aVb = R @ Vb                     # 드론 B의 속도 (프레임 A에서)
        aWb = Wb                         # 2D에서는 각속도는 변환 없이 사용

        # 드론 A의 각속도 (프레임 A에서)
        aWa = self.angularVelocity       # 이미 프레임 A에서 측정된 값으로 가정

        # 드론 A의 속도 (프레임 A에서)
        aVa = self.wRa.T @ self.aVw      # 월드 프레임에서 프레임 A로 변환

        # 위치 업데이트
        aPb_dot = self.vectorToSkewSymmetric(aWa) @ self.state["aPb"] - aVa + aVb
        self.state["aPb"] += aPb_dot * delta_t

        # 자세 업데이트
        omega = (aWb - aWa) * delta_t
        self.state["aRb"] = self.state["aRb"] @ self.exp_map(omega)

    def motionModelJacobian(self):
        R = self.state["aRb"]
        Vb = self.state["Vb"]
        Wb = self.state["Wb"]
        delta_t = self.delta_t

        # 상태 벡터 크기: 5
        Fx = np.eye(5)

        # 속도를 프레임 A로 변환
        aVb = R @ Vb
        aWa = self.angularVelocity  # 각속도는 스칼라

        # 위치 업데이트에 대한 자코비안
        Fx[0:2, 0:2] += self.vectorToSkewSymmetric(aWa) * delta_t  # 각속도에 대한 스큐-대칭 행렬 사용
        # 자세에 대한 편미분 (자세 변화는 스칼라로 처리해야 함, aVb는 2D 벡터)
        Fx[0:2, 3:5] += R * delta_t  # 속도에 대한 편미분 (여기서는 스큐-대칭 행렬이 필요 없음)

        # 자세 업데이트에 대한 자코비안 (각속도에 대한 편미분)
        Fx[2, 2] += -aWa * delta_t
        Fx[2, 4] += delta_t  # 각속도에 대한 편미분

        self.mJacobianMatF = Fx


    def prediction(self):
        self.motionModelJacobian()
        self.motionModel()
        self.matP = self.mJacobianMatF @ self.matP @ self.mJacobianMatF.T + self.matQ * self.delta_t

    def measurementModel(self):
        idx = 0
        for i in range(4):
            for j in range(4):
                # 드론 A의 UWB 노드 위치 (프레임 A에서)
                aPi = self.uwb_positions_drone1[:, i]
                # 드론 B의 UWB 노드 위치를 프레임 A로 변환
                aPj = self.state["aRb"] @ self.uwb_positions_drone2[:, j] + self.state["aPb"]
                # 두 노드 사이의 거리 계산 (2D)
                diff = aPi - aPj
                self.mvecH[idx] = np.linalg.norm(diff)
                idx += 1

    def measurementModelJacobian(self):
        H = np.zeros((16, 5))
        idx = 0
        for i in range(4):
            for j in range(4):
                aPi = self.uwb_positions_drone1[:, i]
                aPj = self.state["aRb"] @ self.uwb_positions_drone2[:, j] + self.state["aPb"]
                diff = aPi - aPj
                dist = np.linalg.norm(diff)

                if dist > 1e-6:
                    # 위치 \( \mathbf{^a\mathbf{p}_b} \)에 대한 편미분
                    H[idx, 0:2] = -diff / dist

                    # 자세 \( \delta \theta \)에 대한 편미분
                    pj_b = self.uwb_positions_drone2[:, j]
                    # 2D 회전의 미분은 벡터 회전과 유사하게 처리 (벡터 pj_b의 회전 미분)
                    J_theta = np.array([[-pj_b[1], pj_b[0]]])  # 회전 미분에 대한 행렬
                    H[idx, 2] = (J_theta @ diff) / dist
                else:
                    H[idx, :] = 0
                idx += 1

        self.mJacobianMatH = H


    def correction(self):
        self.mvecZ = np.array(self.ranges)
        self.measurementModel()
        self.measurementModelJacobian()
        residual = self.mvecZ - self.mvecH

        residualCov = self.mJacobianMatH @ self.matP @ self.mJacobianMatH.T + self.matR
        Kk = self.matP @ self.mJacobianMatH.T @ np.linalg.inv(residualCov)
        stateUpdate = Kk @ residual

        # 상태 업데이트
        self.state["aPb"] += stateUpdate[0:2]
        delta_rot = self.exp_map(stateUpdate[2])
        self.state["aRb"] = self.state["aRb"] @ delta_rot  # 우측 곱 사용
        self.state["Vb"] += stateUpdate[3:5]
        # Wb는 상태 벡터에서 제거하거나 필요 시 업데이트

        # 공분산 업데이트
        self.matP = (np.eye(5) - Kk @ self.mJacobianMatH) @ self.matP

    def publish_pose(self):
        relative_pose_msg = PoseStamped()
        relative_pose_msg.header.stamp = rospy.Time.now()
        relative_pose_msg.header.frame_id = "map"

        # 전역 위치 계산
        global_position = self.wRa @ self.state["aPb"] + self.egoPosition

        relative_pose_msg.pose.position.x = global_position[0]
        relative_pose_msg.pose.position.y = global_position[1]
        relative_pose_msg.pose.position.z = 0.0  # Z축은 추정하지 않으므로 0으로 설정

        # 전역 회전 계산
        global_rotation_angle = self.egoOrientation + np.arctan2(self.state["aRb"][1, 0], self.state["aRb"][0, 0])
        quaternion = tf.transformations.quaternion_from_euler(0, 0, global_rotation_angle)

        relative_pose_msg.pose.orientation.x = quaternion[0]
        relative_pose_msg.pose.orientation.y = quaternion[1]
        relative_pose_msg.pose.orientation.z = quaternion[2]
        relative_pose_msg.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(relative_pose_msg)

if __name__ == "__main__":
    rospy.init_node("RelativePoseEstimation2D")
    rel_pose_estimator = RelativePoseEstimation2D()
    rospy.spin()
