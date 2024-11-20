import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_matrix
import numpy as np
from relative.msg import UwbRange

class uwb_node():
    def __init__(self):
        rospy.init_node('uwb_node')
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.uwb_callback_drone)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.uwb_callback_jackal)
        self.pub_range = rospy.Publisher('/ranges', UwbRange, queue_size=10)
        self.ranges = np.zeros((4, 2))
        self.drone_uwb_position = np.zeros((3, 4))
        self.jackal_uwb_position = np.zeros((3, 2)) 
        self.jackal_pose = None

    def transform_uwb_position(self, pose, local_uwb_position):
        orientation = pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotation_matrix = quaternion_matrix(quaternion)
        local_uwb_position_homogeneous = np.array([*local_uwb_position, 1])
        world_uwb_position = np.dot(rotation_matrix, local_uwb_position_homogeneous)
        uwb_world_x = pose.position.x + world_uwb_position[0]
        uwb_world_y = pose.position.y + world_uwb_position[1]
        uwb_world_z = pose.position.z + world_uwb_position[2]
        return [uwb_world_x, uwb_world_y, uwb_world_z]

    def uwb_callback_drone(self, msg):
        pose = msg.pose
        uwb_positions = [
            [0.13, 0.13, 0],
            [0.13, -0.13, 0],
            [-0.13, 0.13, 0],
            [-0.13, -0.13, 0]
        ]
        for i, uwb_position in enumerate(uwb_positions):
            self.drone_uwb_position[:, i] = self.transform_uwb_position(pose, uwb_position)
        if self.jackal_pose is not None:
            self.calculate_ranges()

    def uwb_callback_jackal(self, msg):
        try:
            index = msg.name.index('jackal')
            pose = msg.pose[index]
            self.jackal_pose = pose
            uwb_positions = [
                [0.3, 0.0, 0],
                [-0.3, 0.0, 0]
            ]
            for i, uwb_position in enumerate(uwb_positions):
                self.jackal_uwb_position[:, i] = self.transform_uwb_position(self.jackal_pose, uwb_position)
            if self.drone_uwb_position is not None:
                self.calculate_ranges()
        except ValueError:
            rospy.logwarn('Jackal model not found in /gazebo/model_states')

    def calculate_ranges(self):
        for i in range(4):
            for j in range(2):
                diff = self.drone_uwb_position[:, i] - self.jackal_uwb_position[:, j]
                distance = np.linalg.norm(diff)
                noisy_distance = distance + np.random.normal(0, 0.07)
                self.ranges[i, j] = noisy_distance
        uwb_range_msg = UwbRange()
        uwb_range_msg.ranges = self.ranges.flatten().tolist()
        self.pub_range.publish(uwb_range_msg)

if __name__ == '__main__':
    node = uwb_node()
    rospy.spin()
