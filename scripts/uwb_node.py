import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix
import numpy as np
from relative.msg import UwbRange

class uwb_node():
    def __init__(self):
        rospy.init_node('uwb_position_transformer')
        rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.uwb_callback_uav0)
        rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, self.uwb_callback_uav1)
        self.pub_range = rospy.Publisher('/ranges', UwbRange, queue_size=10)
        self.ranges = np.zeros((4, 4))
        self.uab0_uwb_position = np.zeros((3, 4))
        self.uab1_uwb_position = np.zeros((3, 4))
        self.noise_mean = 0
        self.noise_std = 0.07

    def transform_uwb_position(self, pose, local_uwb_position):
        orientation = pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotation_matrix = quaternion_matrix(quaternion)
        local_uwb_position_homogeneous = np.array([local_uwb_position[0], local_uwb_position[1], local_uwb_position[2], 1])
        world_uwb_position = np.dot(rotation_matrix, local_uwb_position_homogeneous)
        uwb_world_x = pose.position.x + world_uwb_position[0]
        uwb_world_y = pose.position.y + world_uwb_position[1]
        uwb_world_z = pose.position.z + world_uwb_position[2]

        return [uwb_world_x, uwb_world_y, uwb_world_z]

    def uwb_callback_uav0(self, msg):
        pose = msg.pose
        # uwb_positions = [
        #     [0.17, 0.17, 0],    
        #     [0.17, -0.17, 0],   
        #     [-0.17, 0.17, 0],   
        #     [-0.17, -0.17, 0]   
        # ]
        uwb_positions = [
            [0.5, 0.5, 0],    
            [0.5, -0.5, 0],   
            [-0.5, 0.5, 0],   
            [-0.5, -0.5, 0]   
        ]
        for i, uwb_position in enumerate(uwb_positions):
            self.uab0_uwb_position[:, i] = self.transform_uwb_position(pose, uwb_position)

    def uwb_callback_uav1(self, msg):
        pose = msg.pose
        # uwb_positions = [
        #     [0.17, 0.17, 0],    
        #     [0.17, -0.17, 0],   
        #     [-0.17, 0.17, 0],   
        #     [-0.17, -0.17, 0]   
        # ]
        uwb_positions = [
            [0.5, 0.5, 0],    
            [0.5, -0.5, 0],   
            [-0.5, 0.5, 0],   
            [-0.5, -0.5, 0]   
        ]        
        for i, uwb_position in enumerate(uwb_positions):
            self.uab1_uwb_position[:, i] = self.transform_uwb_position(pose, uwb_position)

        for i in range(4):
            for j in range(4):
                diff = self.uab0_uwb_position[:, i] - self.uab1_uwb_position[:, j]
                distance = np.linalg.norm(diff)
                noise = np.random.normal(self.noise_mean, self.noise_std)
                # self.ranges[i, j] = distance + noise
                self.ranges[i, j] = distance 

        uwb_range_msg = UwbRange()
        uwb_range_msg.ranges = self.ranges.flatten().tolist()  
        self.pub_range.publish(uwb_range_msg)
        
if __name__ == '__main__':
    node = uwb_node()
    rospy.spin()
