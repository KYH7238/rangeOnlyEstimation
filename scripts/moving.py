# import rospy
# from mavros_msgs.srv import SetMode, CommandBool
# from geometry_msgs.msg import PoseStamped
# from mavros_msgs.msg import State
# import math
# import tf

# class MultiDroneController:
#     def __init__(self):
#         self.uav0_state = State()
#         self.uav1_state = State()
#         self.uav0_pose = PoseStamped()
#         self.uav1_pose = PoseStamped()
#         self.time_elapsed = 0  

#         rospy.init_node('multi_drone_takeoff', anonymous=True)

#         rospy.Subscriber("/uav0/mavros/state", State, self.uav0_state_cb)
#         rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.uav0_pose_cb)

#         rospy.Subscriber("/uav1/mavros/state", State, self.uav1_state_cb)
#         rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, self.uav1_pose_cb)

#         while not self.uav0_state.connected or not self.uav1_state.connected:
#             rospy.loginfo("Waiting for drones to connect...")
#             rospy.sleep(1)

#         rospy.loginfo("Drones are connected!")
#         self.manage_drone('uav0')
#         self.manage_drone('uav1')

#         self.control_loop()

#     def uav0_state_cb(self, state):
#         self.uav0_state = state

#     def uav1_state_cb(self, state):
#         self.uav1_state = state

#     def uav0_pose_cb(self, pose):
#         self.uav0_pose = pose

#     def uav1_pose_cb(self, pose):
#         self.uav1_pose = pose

#     def send_setpoint1(self, namespace, x, y, z):
#         pub = rospy.Publisher(f'/{namespace}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
#         pose = PoseStamped()
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.position.z = z
#         rate = rospy.Rate(20)
#         for _ in range(100): 
#             pub.publish(pose)
#             rate.sleep()

#     def send_setpoint(self, namespace, x, y, z, yaw):
#         pub = rospy.Publisher(f'/{namespace}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
#         pose = PoseStamped()
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.position.z = z

#         quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
#         pose.pose.orientation.x = quaternion[0]
#         pose.pose.orientation.y = quaternion[1]
#         pose.pose.orientation.z = quaternion[2]
#         pose.pose.orientation.w = quaternion[3]

#         pub.publish(pose)

#     def arm_drone(self, namespace):
#         rospy.wait_for_service(f'/{namespace}/mavros/cmd/arming')
#         try:
#             arm_srv = rospy.ServiceProxy(f'/{namespace}/mavros/cmd/arming', CommandBool)
#             response = arm_srv(True)
#             if response.success:
#                 rospy.loginfo(f"{namespace} armed successfully")
#             else:
#                 rospy.logwarn(f"{namespace} arm failed")
#         except rospy.ServiceException as e:
#             rospy.logerr(f"Arm service call failed for {namespace}: {e}")

#     def set_mode(self, namespace, mode):
#         rospy.wait_for_service(f'/{namespace}/mavros/set_mode')
#         try:
#             set_mode_srv = rospy.ServiceProxy(f'/{namespace}/mavros/set_mode', SetMode)
#             response = set_mode_srv(0, mode)
#             if response.mode_sent:
#                 rospy.loginfo(f"{namespace} set to {mode} mode successfully")
#             else:
#                 rospy.logwarn(f"{namespace} set mode failed")
#         except rospy.ServiceException as e:
#             rospy.logerr(f"Set mode service call failed for {namespace}: {e}")

#     def manage_drone(self, namespace):
#         self.send_setpoint1(namespace, 0, 0, 0) 
#         self.set_mode(namespace, 'OFFBOARD')
#         self.arm_drone(namespace)
#         rospy.loginfo(f"{namespace} moving to 5 meters altitude")

#     def control_loop(self):
#         rate = rospy.Rate(20)
#         while not rospy.is_shutdown():
#             self.time_elapsed += 0.02  

#             #8자
#             x_uav0 = 5 * math.sin(self.time_elapsed)
#             y_uav0 = 5 * math.sin(2 * self.time_elapsed)
#             yaw_uav0 = math.atan2(2 * math.cos(2 * self.time_elapsed), math.cos(self.time_elapsed)) 
#             self.send_setpoint('uav0', x_uav0, y_uav0, 5, yaw_uav0)

#             #원형 
#             x_uav1 = 5 * math.cos(self.time_elapsed)
#             y_uav1 = 5 * math.sin(self.time_elapsed)
#             yaw_uav1 = math.atan2(math.sin(self.time_elapsed), math.cos(self.time_elapsed)) 
#             self.send_setpoint('uav1', x_uav1, y_uav1, 5, yaw_uav1)

#             rate.sleep()

# if __name__ == "__main__":
#     # rospy.sleep(10)
#     controller = MultiDroneController()



import rospy
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import math
import tf

class SingleDroneController:
    def __init__(self):
        self.drone_state = State()
        self.drone_pose = PoseStamped()
        self.time_elapsed = 0  

        rospy.init_node('single_drone_trajectory', anonymous=True)

        # Subscribers
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)

        # Wait for the drone to connect
        while not self.drone_state.connected:
            rospy.loginfo("Waiting for the drone to connect...")
            rospy.sleep(1)

        rospy.loginfo("Drone is connected!")
        self.manage_drone()
        self.control_loop()

    def state_cb(self, state):
        self.drone_state = state

    def pose_cb(self, pose):
        self.drone_pose = pose

    def send_setpoint(self, x, y, z, yaw):
        pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        pub.publish(pose)

    def arm_drone(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arm_srv(True)
            if response.success:
                rospy.loginfo("Drone armed successfully")
            else:
                rospy.logwarn("Drone arm failed")
        except rospy.ServiceException as e:
            rospy.logerr(f"Arm service call failed: {e}")

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_srv(0, mode)
            if response.mode_sent:
                rospy.loginfo(f"Drone set to {mode} mode successfully")
            else:
                rospy.logwarn("Set mode failed")
        except rospy.ServiceException as e:
            rospy.logerr(f"Set mode service call failed: {e}")

    def manage_drone(self):
        # Send a few initial setpoints to start offboard mode
        self.send_initial_setpoints()
        self.set_mode('OFFBOARD')
        self.arm_drone()
        rospy.loginfo("Drone moving to 5 meters altitude")

    def send_initial_setpoints(self):
        pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        rate = rospy.Rate(20)
        for _ in range(100):
            pub.publish(pose)
            rate.sleep()

    def control_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.time_elapsed += 0.02  

            # Calculate 8-shaped trajectory
            x = 5 * math.sin(self.time_elapsed)
            y = 5 * math.sin(2 * self.time_elapsed)
            yaw = math.atan2(2 * math.cos(2 * self.time_elapsed), math.cos(self.time_elapsed)) 
            self.send_setpoint(x, y, 5, yaw)

            rate.sleep()

if __name__ == "__main__":
    controller = SingleDroneController()
