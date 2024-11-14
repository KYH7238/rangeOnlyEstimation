import rospy
import numpy as np
from relative.msg import UwbRange
from geometry_msgs.msg import Pose2D, PoseStamped
from tf.transformations import euler_from_quaternion

class EKFNode:
    def __init__(self):
        rospy.init_node('ekf_node')
        
        # STATE = [x_ij, y_ij, theta_ij, v_j, w_j].T
        self.x = np.zeros((5, 1)) 
        self.P = np.eye(5) * 0.1   
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        self.R = np.eye(8) * 0.05  
        self.dt = 0.0035 
        self.drone_pose = PoseStamped()
        self.drone_uwb_positions = np.array([
            [0.13, 0.13, 0],
            [0.13, -0.13, 0],
            [-0.13, 0.13, 0],
            [-0.13, -0.13, 0]
        ])
        self.jackal_uwb_positions = np.array([
            [0.25, 0.0, 0],
            [-0.25, 0.0, 0]
        ])
        rospy.Subscriber('/ranges', UwbRange, self.uwb_callback)
        self.state_pub = rospy.Publisher('/estimated_state', Pose2D, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.prev_time = rospy.Time.now()

    def pose_cb(self, pose):
        self.drone_pose = pose

    def predict(self):
        x = self.x
        dt = self.dt

        yaw = x[2, 0]
        v_j = x[3, 0]
        omega_j = x[4, 0]
        
        x_pred = np.zeros((5, 1))
        x_pred[0, 0] = x[0, 0] + v_j * dt * np.cos(yaw)
        x_pred[1, 0] = x[1, 0] + v_j * dt * np.sin(yaw)
        x_pred[2, 0] = x[2, 0] + omega_j * dt
        x_pred[3, 0] = x[3, 0]
        x_pred[4, 0] = x[4, 0]

        F = np.array([
            [1, 0, -v_j * dt * np.sin(yaw), dt * np.cos(yaw), 0],
            [0, 1,  v_j * dt * np.cos(yaw), dt * np.sin(yaw), 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])
        
        P_pred = F.dot(self.P).dot(F.T) + self.Q
        
        self.x = x_pred
        self.P = P_pred
        
    def update(self, z):
        x = self.x
        m = z.shape[0]
        h = np.zeros((m, 1))
        H = np.zeros((m, 5))
        
        idx = 0
        for i in range(4):
            p_i = self.drone_uwb_positions[i, :]
            for j in range(2): 
                p_j = self.jackal_uwb_positions[j, :]
                theta = x[2, 0]
                R = np.array([
                    [np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta),  np.cos(theta), 0],
                    [0, 0, 1]
                ])
                
                p_j_in_drone = R.dot(p_j) + np.array([x[0, 0], x[1, 0], 0])
                diff = p_i - p_j_in_drone
                h[idx, 0] = np.linalg.norm(diff)
                dist = h[idx, 0]
                if dist == 0:
                    dist = 1e-4  
                
                dh_dx = np.zeros((1, 5))
                dh_dx[0, 0] = (p_j_in_drone[0] - p_i[0]) / dist
                dh_dx[0, 1] = (p_j_in_drone[1] - p_i[1]) / dist
                dh_dx[0, 2] = ((-np.sin(theta) * p_j[0] - np.cos(theta) * p_j[1]) * (p_j_in_drone[0] - p_i[0]) + 
                               ( np.cos(theta) * p_j[0] - np.sin(theta) * p_j[1]) * (p_j_in_drone[1] - p_i[1])) / dist
                dh_dx[0, 3] = 0  
                dh_dx[0, 4] = 0  
                
                H[idx, :] = dh_dx
                idx += 1
        y = z - h
        S = H.dot(self.P).dot(H.T) + self.R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        self.P = (np.eye(5) - K.dot(H)).dot(self.P)
        
    def uwb_callback(self, msg):
        z = np.array(msg.ranges).reshape(-1, 1)
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        if dt > 0:
            self.dt = dt
        self.prev_time = current_time
        self.predict()
        self.update(z)
        estimated_pose = Pose2D()
        estimated_pose.x = self.x[0, 0] 
        estimated_pose.y = self.x[1, 0] 
        estimated_pose.theta = self.x[2, 0]
        self.state_pub.publish(estimated_pose)
        
if __name__ == '__main__':
    try:
        ekf_node = EKFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# import rospy
# import numpy as np
# from relative.msg import UwbRange
# from geometry_msgs.msg import Pose2D, PoseStamped, TwistStamped
# from tf.transformations import euler_from_quaternion

# class EKFNode:
#     def __init__(self):
#         rospy.init_node('ekf_node')
        
#         # STATE = [x_ij, y_ij, theta_ij, v_j, w_j].T
#         self.x = np.zeros((5, 1))  
#         self.P = np.eye(5) * 0.1   
#         self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
#         self.R = np.eye(8) * 0.05  
#         self.dt = 0.0035  
#         self.drone_pose = PoseStamped()
#         self.drone_velocity = TwistStamped()
        
#         self.drone_uwb_positions = np.array([
#             [0.13, 0.13, 0],
#             [0.13, -0.13, 0],
#             [-0.13, 0.13, 0],
#             [-0.13, -0.13, 0]
#         ])
#         self.jackal_uwb_positions = np.array([
#             [0.25, 0.0, 0],
#             [-0.25, 0.0, 0]
#         ])
        
#         rospy.Subscriber('/ranges', UwbRange, self.uwb_callback)
#         rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
#         rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.velocity_cb)
        
#         self.state_pub = rospy.Publisher('/estimated_state', Pose2D, queue_size=10)
#         self.prev_time = rospy.Time.now()

#     def pose_cb(self, pose):
#         self.drone_pose = pose

#     def velocity_cb(self, velocity):
#         self.drone_velocity = velocity

#     def predict(self):
#         x = self.x
#         dt = self.dt

#         # Get the yaw (orientation) of the ego drone
#         orientation_q = self.drone_pose.pose.orientation
#         _, _, drone_yaw = euler_from_quaternion([
#             orientation_q.x,
#             orientation_q.y,
#             orientation_q.z,
#             orientation_q.w
#         ])
        
#         # Velocity of the ego drone in its frame
#         v_i = self.drone_velocity.twist.linear.x
#         omega_i = self.drone_velocity.twist.angular.z
        
#         # Relative state prediction incorporating ego drone motion
#         yaw_j = x[2, 0]
#         v_j = x[3, 0]
#         omega_j = x[4, 0]
        
#         x_pred = np.zeros((5, 1))
#         x_pred[0, 0] = x[0, 0] + (v_j - v_i) * dt * np.cos(yaw_j - drone_yaw)
#         x_pred[1, 0] = x[1, 0] + (v_j - v_i) * dt * np.sin(yaw_j - drone_yaw)
#         x_pred[2, 0] = x[2, 0] + (omega_j - omega_i) * dt
#         x_pred[3, 0] = x[3, 0]
#         x_pred[4, 0] = x[4, 0]

#         F = np.array([
#             [1, 0, -(v_j - v_i) * dt * np.sin(yaw_j - drone_yaw), dt * np.cos(yaw_j - drone_yaw), 0],
#             [0, 1,  (v_j - v_i) * dt * np.cos(yaw_j - drone_yaw), dt * np.sin(yaw_j - drone_yaw), 0],
#             [0, 0, 1, 0, dt],
#             [0, 0, 0, 1, 0],
#             [0, 0, 0, 0, 1]
#         ])
        
#         P_pred = F.dot(self.P).dot(F.T) + self.Q
        
#         self.x = x_pred
#         self.P = P_pred

#     def update(self, z):
#         x = self.x
#         m = z.shape[0]
#         h = np.zeros((m, 1))
#         H = np.zeros((m, 5))
        
#         # Adjust for ego drone pose (position and orientation)
#         orientation_q = self.drone_pose.pose.orientation
#         _, _, drone_yaw = euler_from_quaternion([
#             orientation_q.x,
#             orientation_q.y,
#             orientation_q.z,
#             orientation_q.w
#         ])
        
#         idx = 0
#         for i in range(4):
#             p_i = self.drone_uwb_positions[i, :]
#             for j in range(2): 
#                 p_j = self.jackal_uwb_positions[j, :]
                
#                 # Adjust theta with respect to the drone's current yaw
#                 theta = x[2, 0] + drone_yaw  
                
#                 R = np.array([
#                     [np.cos(theta), -np.sin(theta), 0],
#                     [np.sin(theta),  np.cos(theta), 0],
#                     [0, 0, 1]
#                 ])
                
#                 p_j_in_drone = R.dot(p_j) + np.array([x[0, 0], x[1, 0], 0])
#                 diff = p_i - p_j_in_drone
#                 h[idx, 0] = np.linalg.norm(diff)
#                 dist = h[idx, 0]
#                 if dist == 0:
#                     dist = 1e-4  
                
#                 dh_dx = np.zeros((1, 5))
#                 dh_dx[0, 0] = (p_j_in_drone[0] - p_i[0]) / dist
#                 dh_dx[0, 1] = (p_j_in_drone[1] - p_i[1]) / dist
#                 dh_dx[0, 2] = ((-np.sin(theta) * p_j[0] - np.cos(theta) * p_j[1]) * (p_j_in_drone[0] - p_i[0]) + 
#                                ( np.cos(theta) * p_j[0] - np.sin(theta) * p_j[1]) * (p_j_in_drone[1] - p_i[1])) / dist
#                 dh_dx[0, 3] = 0  
#                 dh_dx[0, 4] = 0  
                
#                 H[idx, :] = dh_dx
#                 idx += 1
        
#         y = z - h
#         S = H.dot(self.P).dot(H.T) + self.R
#         K = self.P.dot(H.T).dot(np.linalg.inv(S))
#         self.x = self.x + K.dot(y)
#         self.P = (np.eye(5) - K.dot(H)).dot(self.P)

#     def uwb_callback(self, msg):
#         z = np.array(msg.ranges).reshape(-1, 1)
#         current_time = rospy.Time.now()
#         dt = (current_time - self.prev_time).to_sec()
#         if dt > 0:
#             self.dt = dt
#         self.prev_time = current_time
#         self.predict()
#         self.update(z)
#         estimated_pose = Pose2D()
#         estimated_pose.x = self.x[0, 0]
#         estimated_pose.y = self.x[1, 0]
#         estimated_pose.theta = self.x[2, 0]
#         self.state_pub.publish(estimated_pose)

# if __name__ == '__main__':
#     try:
#         ekf_node = EKFNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
