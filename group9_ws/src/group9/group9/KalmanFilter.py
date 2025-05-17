import numpy as np
from geometry_msgs.msg import Pose
class KalmanFilter:
    def __init__(self):
        self.x = np.zeros((3,1))
        self.q = np.zeros(4)
        self.sigma = np.eye(3)*10.0 #Initial uncertainty (map of 10m?)

    def updatePosition(self, yt):
        #Motion model
        # x_{t+1} = x_t + v_t

        #Sensor model
        #y_t = C x_t + w
        A = np.eye(3)
        C = np.eye(3)
        R = np.eye(3)*0.0001 #Process noise (very low)
        Q = np.eye(3)*0.05 #Measurement noise

        mu_bar = A@self.x.reshape(3,1)
        sigma_bar = A@self.sigma@A.T + R

        K = sigma_bar@C.T@np.linalg.inv(C@sigma_bar@C.T + Q)
        self.x = (mu_bar + K@(yt - C@mu_bar))
        self.sigma = (np.eye(3)- K@C)@sigma_bar
    
    def updateRotation(self, qt, alpha = 0.05):
        # Rotations are not linear => need a EKF, for simplicity, we will consider weighted average for updating
        # The camera is better in orientation than in position
        # First intuition is therefore to only average update the quaternions
        q = (1- alpha)*self.q + alpha*qt
        self.q = q/np.linalg.norm(q)
        
    
    def updateWithPose(self,pose):
        yt = [pose.position.x, pose.position.y]
        qt = [pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w]
        self.updatePosition(self,yt)
        self.updateRotation(self,qt)
    
    def get_estimate(self):
        p = Pose()
        p.position.x = self.x[0]
        p.position.y = self.x[1]
        p.position.z = self.x[1]
        
        p.orientation.x = self.q[0]
        p.orientation.y = self.q[1]
        p.orientation.z = self.q[2]
        p.orientation.w = self.q[3]
        return p
