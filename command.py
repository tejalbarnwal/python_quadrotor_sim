# creates commands for quad
# gives the setpoints for the drone so known as commander

import numpy as np


class commander():
	def __init__(self):
		self.position = None
		self.orientation = None
		self.v = None
		self.omega = None

	def position(self, pose):
		self.position = pose

	def orientation(self, orientation):
		self.orientation = orientation

	def velocity(self, v):
		self.v = v

	def angular_velocity(self, w):
		self.omega = w

	def get_command(self):
		pos = self.position
        Phi = self.orientation
        commanded = np.hstack((pos, self.v, Phi, self.omega))
        return commanded
