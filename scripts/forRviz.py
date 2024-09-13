import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class node():
    def __init__(self):
        rospy.init_node("ParticleFilterNode")
        self.droneX = 0
        self.droneY = 0
        self.droneZ = 0
        self.turtlebotArrayX = [] 
        self.turtlebotArrayY = [] 
        self.droneArrayX = []
        self.droneArrayY = []
        self.turtlebotX = 0
        self.turtlebotY = 0
        self.cnt_1 = 0
        self.cnt_2 = 0
        self.distanceArray = []  
        self.dronePositionArray = []  
        self.estimated_position = (0, 0)  
        self.estimated_positionArrayX = []  
        self.estimated_positionArrayY = []  

        # Path data
        self.path = Path()
        self.path.header.frame_id = "map"

        # Subscribers and Publishers
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.dronePosition)
        rospy.Subscriber("/odom", Odometry, self.turtlebotPosition)
        self.pub_distance = rospy.Publisher("/distance", PoseStamped, queue_size=10)
        self.pub_pcl = rospy.Publisher("/particle_cloud", PointCloud2, queue_size=10)  # PointCloud2 publisher
        self.pub_path = rospy.Publisher("/turtlebot_path", Path, queue_size=10)  # Path publisher

        self.pf = particleFilter()  
    
    def dronePosition(self, msg):
        self.droneX = msg.pose.position.x
        self.droneY = msg.pose.position.y
        self.droneZ = msg.pose.position.z
        self.cnt_2 += 1
        if self.cnt_2 == 3:
            self.droneArrayX.append(self.droneX)  
            self.droneArrayY.append(self.droneY) 
            self.cnt_2 = 0
    
    def turtlebotPosition(self, msg):
        self.cnt_1 += 1
        self.turtlebotX = msg.pose.pose.position.x
        self.turtlebotY = msg.pose.pose.position.y

        # Add odom position to the Path
        if self.cnt_1 == 3:
            self.turtlebotArrayX.append(self.turtlebotX)  
            self.turtlebotArrayY.append(self.turtlebotY)
            
            # Create a PoseStamped for the path
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.turtlebotX
            pose.pose.position.y = self.turtlebotY
            pose.pose.position.z = 0.0  # Assuming a 2D path
            pose.pose.orientation = msg.pose.pose.orientation  # Keeping the same orientation
            
            # Add the pose to the path and publish
            self.path.poses.append(pose)
            self.pub_path.publish(self.path)

            self.cnt_1 = 0
        
        self.calDistance()

    def calDistance(self):
        distance = np.sqrt((self.turtlebotX - self.droneX) ** 2 + 
                           (self.turtlebotY - self.droneY) ** 2 + 
                            self.droneZ ** 2 +
        np.random.normal(0, 0.05))
        dis = PoseStamped()
        dis.pose.position.x = distance
        self.pub_distance.publish(dis)
         
        self.dronePositionArray.append((self.droneX, self.droneY, self.droneZ))
        self.distanceArray.append(distance)

        if len(self.distanceArray) == 150:
            self.pf.run_particle_filter(self.distanceArray, self.dronePositionArray)
            self.estimated_position = self.pf.estimate()
            
            self.estimated_positionArrayX.append(self.estimated_position[0])
            self.estimated_positionArrayY.append(self.estimated_position[1])

            self.publish_point_cloud(self.pf.particles, self.pf.weights)
            
            self.distanceArray.pop(0)
            self.dronePositionArray.pop(0)

    def publish_point_cloud(self, particles, weights):

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        points = []
        for particle, weight in zip(particles, weights):
            points.append([particle[0], particle[1], 0.0, weight])  
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, points)

        self.pub_pcl.publish(cloud_msg)

    def plot_data(self):
        plt.ion() 
        plt.figure()

        rate = rospy.Rate(10)  

        while not rospy.is_shutdown():
            plt.clf()  
            plt.scatter(self.turtlebotArrayX, self.turtlebotArrayY, c='b', label='Turtlebot (odom)', s=10)
            plt.scatter(self.estimated_positionArrayX, self.estimated_positionArrayY, c='r', label='Estimated Position (PF)', s=10)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.legend()
            plt.pause(0.1) 
            rate.sleep()  


class particleFilter():
    def __init__(self, num_particles=70000):
        self.num_particles = num_particles
        self.particles = np.random.uniform(-10, 10, (self.num_particles, 2))  
        self.weights = np.ones(self.num_particles) / self.num_particles  
    
    def predict(self):
        self.particles += np.random.normal(-1, 1, (self.num_particles, 2))
        
    def update_weights(self, distances, drone_positions):
        total_weights = np.ones(self.num_particles)
        for distance, drone_position in zip(distances, drone_positions):
            distances_from_drone = np.sqrt((self.particles[:, 0] - drone_position[0]) ** 2 +
                                           (self.particles[:, 1] - drone_position[1]) ** 2 +
                                           drone_position[2] ** 2)
            weight = np.exp(-((distances_from_drone - distance) ** 2) / (2 * 0.5 ** 2))
            total_weights *= weight  
        total_weights += 1.e-300  
        self.weights = total_weights / np.sum(total_weights) 

    def resample(self):
        indices = np.random.choice(self.num_particles, self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles) 

    def estimate(self):
        return np.average(self.particles, axis=0, weights=self.weights)

    def run_particle_filter(self, distances, drone_positions):
        self.predict()  
        self.update_weights(distances, drone_positions)  
        self.resample()  

if __name__ == "__main__":
    pf_node = node()
    pf_node.plot_data()
    rospy.spin()
