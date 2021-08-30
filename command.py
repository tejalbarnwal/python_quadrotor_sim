# creates commands for quad
# gives the setpoints for the drone so known as commander

from derivative_lpf import DirtyDerivative
import numpy as np


class commander():
    def __init__(self):
        self.position = np.array([0.0,0.0,0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])
        self.v = np.array([0.0,0.0,0.0])
        self.omega = np.array([0, 0, 0])

    def position(self, pose):
        self.position = pose

    def orientation(self, orientation):
        self.orientation = orientation

    def velocity(self, v):
        self.v = v

    def angular_velocity(self, w):
        self.omega = w

    def get_command(self):
        commanded = np.hstack((self.position, self.v, self.orientation, self.omega))
        return commanded


# commander = commander()
# setpoints = commander.get_command()
# print(setpoints)
