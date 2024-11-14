import rospy
from geometry_msgs.msg import Twist

class JackalCircleMover:
    def __init__(self):
        rospy.init_node('jackal_circle_mover')
        self.cmd_vel_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.linear_speed = 0.5  
        self.angular_speed = 0.5 
        self.rate = rospy.Rate(10)
        self.run()

    def run(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = self.angular_speed
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        mover = JackalCircleMover()
    except rospy.ROSInterruptException:
        pass
