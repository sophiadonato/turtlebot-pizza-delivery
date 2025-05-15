import numpy as np
from geometry_msgs.msg import Pose
class KalmanFilter:
    def __init__(self):
        self.x = np.zeros((2,1))
        self.sigma = np.eye(2)*10.0 #Initial uncertainty (map of 10m?)

    def update(self, yt):
        #Motion model
        # x_{t+1} = x_t + v_t

        #Sensor model
        #y_t = C x_t + w
        A = np.eye(2)
        C = np.eye(2)
        R = np.eye(2)*0.0001 #Process noise (very low)
        Q = np.eye(2)*0.05 #Measurement noise

        mu_bar = A@self.x.reshape(2,1)
        sigma_bar = A@self.sigma@A.T + R

        K = sigma_bar@C.T@np.linalg.inv(C@sigma_bar@C.T + Q)
        self.x = (mu_bar + K@(yt - C@mu_bar))
        self.sigma = (np.eye(2)- K@C)@sigma_bar
    
    def updateWithPose(self,pose):
        yt = [pose.position.x, pose.position.y]
        self.update(self,yt)

    
    def get_estimate(self):
        return self.x.flatten
