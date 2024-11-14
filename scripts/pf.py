import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
import numpy as np
from relative.msg import UwbRange

class ParticleFilterRelativePoseEstimation():
    def __init__(self, num_particles=100):
        self.num_particles = num_particles

        # 상태: [x, y, z (z는 고정된 5), R(회전 행렬), Vb(상대 속도), Wb(상대 각속도)]
        self.particles = {
            "positions": np.random.uniform(-5, 5, (num_particles, 3)),  # x, y, z 좌표 초기화
            "orientations": [np.eye(3) for _ in range(num_particles)],  # 회전 행렬 초기화
            "Vb": np.zeros((num_particles, 3)),  # 상대 속도
            "Wb": np.zeros((num_particles, 3)),  # 상대 각속도
        }
        # self.particles["positions"][:, 2] = 5  # z 좌표 고정

        self.weights = np.ones(num_particles) / num_particles

        # 측정 노이즈 공분산 정의
        self.matR = 0.1 * np.eye(16, dtype=np.float32)  # 측정 노이즈 공분산 (대략적인 값 설정)

        rospy.Subscriber("/ranges", UwbRange, self.uwbCallback)
        rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.egoPoseCallback)
        rospy.Subscriber("/uav0/mavros/local_position/velocity_local", TwistStamped, self.velocityCallback)
        rospy.Subscriber("/uav0/mavros/imu/data", Imu, self.imuCallback)
        self.pose_pub = rospy.Publisher("/relative_pose", PoseStamped, queue_size=10)
        self.egoPosition = np.zeros(3)
        self.linearVelocity = np.zeros(3)
        self.angularVelocity = np.zeros(3)
        self.delta_t = 0.01
        self.orientation = None
        self.before_t = rospy.Time.now()
        self.uwb_positions_drone1 = np.array([
            [0.5, 0.5, 0],    
            [0.5, -0.5, 0],   
            [-0.5, 0.5, 0],   
            [-0.5, -0.5, 0]
        ]).T
        self.uwb_positions_drone2 = self.uwb_positions_drone1
        # self.uwb_positions_drone2[2, :] += 5  # 상대 드론의 z 고정

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
            self.linearVelocity = np.zeros(3)

    def imuCallback(self, msg):
        self.angularVelocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def world_to_body_velocity(self, world_velocity):
        quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        self.wRa = tf.transformations.quaternion_matrix(quaternion)[:3, :3] 
        body_velocity = np.dot(self.wRa.T, world_velocity) 
        return body_velocity

    def process_data(self):
        if len(self.range) != 16:
            return  # 데이터가 충분하지 않으면 반환
        self.mvecZ = np.array(self.range)
        current_time = rospy.Time.now()
        self.delta_t = (current_time - self.before_t).to_sec()
        if self.delta_t == 0:
            self.delta_t = 0.01  # 시간 간격이 0이면 기본값 설정
        self.before_t = current_time
        self.prediction()
        self.correction()
        self.resample()
        self.publish_pose()

    def exp_map(self, omega):
        angle = np.linalg.norm(omega)
        if angle < 1e-9:
            return np.eye(3)
        axis = omega / angle
        K = self.vectorToSkewSymmetric(axis)
        return np.cos(angle) * np.eye(3) + (1 - np.cos(angle)) * np.outer(axis, axis) + np.sin(angle) * K

    def vectorToSkewSymmetric(self, vec):
        return np.array([[0, -vec[2], vec[1]],
                         [vec[2], 0, -vec[0]],
                         [-vec[1], vec[0], 0]], dtype=np.float32)

    def prediction(self):
        # 파티클의 상태 예측 단계
        for i in range(self.num_particles):
            R = self.particles["orientations"][i]
            Vb = self.particles["Vb"][i]
            Wb = self.particles["Wb"][i]

            # 위치 업데이트 (z 축 고정)
            position_dot = self.vectorToSkewSymmetric(self.angularVelocity) @ self.particles["positions"][i] - self.linearVelocity + R @ Vb
            self.particles["positions"][i] += position_dot * self.delta_t
            # z 좌표 고정
            self.particles["positions"][i, 2] = 5

            # 회전 행렬 업데이트
            omega = (R @ Wb - self.angularVelocity) * self.delta_t
            self.particles["orientations"][i] = R @ self.exp_map(omega)

    def correction(self):
        # 파티클의 가중치 업데이트 단계
        for i in range(self.num_particles):
            estimated_ranges = self.measurement_model(self.particles["positions"][i], self.particles["orientations"][i])
            self.weights[i] = self.calculate_likelihood(self.mvecZ, estimated_ranges)

        # 가중치 정규화
        self.weights += 1.e-300  # 제로 디비전 방지용
        self.weights /= np.sum(self.weights)

    def measurement_model(self, position, orientation):
        # 측정 모델: 주어진 파티클의 위치와 자세에 대해 예측된 UWB 거리를 계산
        idx = 0
        predicted_ranges = np.zeros(16)
        for i in range(4):
            for j in range(4):
                sensor_1_position = orientation @ self.uwb_positions_drone1[:, i] + position
                sensor_2_position = self.uwb_positions_drone2[:, j]
                diff = sensor_1_position - sensor_2_position
                predicted_ranges[idx] = np.linalg.norm(diff)
                idx += 1
        return predicted_ranges

    def calculate_likelihood(self, actual_ranges, predicted_ranges):
        # 실제 측정값과 예측된 측정값의 차이를 통해 가중치를 계산 (여기서는 가우시안 분포 기반)
        error = actual_ranges - predicted_ranges
        sigma_squared = self.matR[0, 0]  # 가우시안 노이즈 분산
        likelihood = np.exp(-0.5 * np.sum(error ** 2) / sigma_squared)  # 대략적인 노이즈 모델 적용
        return likelihood

    def resample(self):
        # 중요도 재샘플링 단계
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # 가능성 오차 방지
        indexes = np.searchsorted(cumulative_sum, np.random.uniform(0, 1, self.num_particles))

        # 파티클 재샘플링
        self.particles["positions"] = self.particles["positions"][indexes]
        self.particles["orientations"] = [self.particles["orientations"][i] for i in indexes]
        self.particles["Vb"] = self.particles["Vb"][indexes]
        self.particles["Wb"] = self.particles["Wb"][indexes]

        # 가중치 초기화
        self.weights.fill(1.0 / self.num_particles)

    def publish_pose(self):
        # 파티클 필터의 평균 위치와 자세를 계산하여 퍼블리시
        mean_position = np.average(self.particles["positions"], axis=0, weights=self.weights)
        mean_orientation = np.mean(self.particles["orientations"], axis=0)

        relative_pose_msg = PoseStamped()
        relative_pose_msg.header.stamp = rospy.Time.now()
        relative_pose_msg.header.frame_id = "map"

        # 전역 위치 계산
        global_position = self.wRa @ mean_position + self.egoPosition

        relative_pose_msg.pose.position.x = global_position[0]
        relative_pose_msg.pose.position.y = global_position[1]
        relative_pose_msg.pose.position.z = global_position[2]

        # 전역 회전 행렬 계산
        global_rotation_matrix = self.wRa @ mean_orientation
        quaternion = tf.transformations.quaternion_from_matrix(
            np.vstack((np.hstack((global_rotation_matrix, np.array([[0], [0], [0]]))), np.array([0, 0, 0, 1])))
        )
        relative_pose_msg.pose.orientation.x = quaternion[0]
        relative_pose_msg.pose.orientation.y = quaternion[1]
        relative_pose_msg.pose.orientation.z = quaternion[2]
        relative_pose_msg.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(relative_pose_msg)

if __name__ == "__main__":
    rospy.init_node("ParticleFilterRelativePoseEstimation")
    Rel = ParticleFilterRelativePoseEstimation()
    rospy.spin()
