import rospy
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

# 드론의 상태와 위치를 저장할 전역 변수
uav0_state = State()
uav1_state = State()
uav0_pose = PoseStamped()
uav1_pose = PoseStamped()


def uav0_state_cb(state):
    global uav0_state
    uav0_state = state

def uav1_state_cb(state):
    global uav1_state
    uav1_state = state
    send_setpoint('uav1', 0, 0, 5)

def uav0_pose_cb(pose):
    global uav0_pose
    uav0_pose = pose
    send_setpoint('uav0', 0, 0, 5)

def uav1_pose_cb(pose):
    global uav1_pose
    uav1_pose = pose


def send_setpoint(namespace, x, y, z):
    pub = rospy.Publisher(f'/{namespace}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    rate = rospy.Rate(20)
    for _ in range(100): 
        pub.publish(pose)
        rate.sleep()


def arm_drone(namespace):
    rospy.wait_for_service(f'/{namespace}/mavros/cmd/arming')
    try:
        arm_srv = rospy.ServiceProxy(f'/{namespace}/mavros/cmd/arming', CommandBool)
        response = arm_srv(True)
        if response.success:
            rospy.loginfo(f"{namespace} armed successfully")
        else:
            rospy.logwarn(f"{namespace} arm failed")
    except rospy.ServiceException as e:
        rospy.logerr(f"Arm service call failed for {namespace}: {e}")

def set_mode(namespace, mode):
    rospy.wait_for_service(f'/{namespace}/mavros/set_mode')
    try:
        set_mode_srv = rospy.ServiceProxy(f'/{namespace}/mavros/set_mode', SetMode)
        response = set_mode_srv(0, mode)
        if response.mode_sent:
            rospy.loginfo(f"{namespace} set to {mode} mode successfully")
        else:
            rospy.logwarn(f"{namespace} set mode failed")
    except rospy.ServiceException as e:
        rospy.logerr(f"Set mode service call failed for {namespace}: {e}")

def manage_drone(namespace):
    send_setpoint(namespace, 0, 0, 0)
    set_mode(namespace, 'OFFBOARD')
    arm_drone(namespace)


    rospy.loginfo(f"{namespace} moving to 5 meters altitude")
    # send_setpoint(namespace, 0, 0, 5)

def main():
    rospy.init_node('multi_drone_takeoff', anonymous=True)


    rospy.Subscriber("/uav0/mavros/state", State, uav0_state_cb)
    rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, uav0_pose_cb)
    

    rospy.Subscriber("/uav1/mavros/state", State, uav1_state_cb)
    rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, uav1_pose_cb)

    while not uav0_state.connected or not uav1_state.connected:
        rospy.loginfo("Waiting for drones to connect...")
        rospy.sleep(1)

    rospy.loginfo("Drones are connected!")
    manage_drone('uav0')
    manage_drone('uav1')




    rospy.spin()

if __name__ == "__main__":
    main()
