import rospy
import numpy as np

class EKF():
    def __init__(self) -> None:
        self.groundRobotState = np.zeros(4) # Position X, Y // Velocity X, Y
        self.droneState = np.zeros(3) # Position X, Y, Z
        self.matQ= np.eye(4)
        self.matR = np.ones(1)
        self.motionModelJacobian = np.eye(4)
        self.measurementModelJacobian = np.zeros(3,1)
        self.deltaT= 0
    
    def motionModelJacobian(self):
        self.motionModelJacobian[2,2] = self.motionModelJacobian[3,3] = self.deltaT
         
    def prediction(self):
        self.groundRobotState[0] += self.groundRobotState[2]*self.deltaT
        self.groundRobotState[1] += self.groundRobotState[3]*self.deltaT

    def measurementModelJacobian(self):
        pass

    def correction(self):
        pass



if __name__ =="__main__":
    ekf = EKF()