import numpy as np

class Simulator():
	def __init__(self, quad, ctrl, cmdr):
		# define inputs
		self.quadrotor_model = quad
		self.controller = ctrl
		self.command = cmdr

		# define history
		self.history = {}

		# simulation parameters
		self.Tf = 0.0
		self.Ts = 0.0
		self.N = 0.0


	def __str__(self):
		s = "Simulator"
		return s

	def run(self, Tf, Ts=0.01):
		# run all the simulation and store the data to plot in history

		#save the simulation parameters
		self.Tf = Tf
		self.Ts = Ts
		# save how many iterations of update would be needed
		self.N = int(Tf/Ts)
		# quadrotor initial state
		truth = np.zeros((12,1))
		truth[0:3] = self.quadrotor_model.position
		truth[3:6] = self.quadrotor_model.orientation
		truth[6:9] = self.quadrotor_model.vel
		truth[9:12] = self.quadrotor_model.angular_vel

		# define history components
		self.history["u"] = np.zeros((4,self.N))
		self.history["commands"] = np.zeros((12, self.N))
		self.history["truth"] = np.zeros((12, self.N))

		# Now simulation starts
		for i in range(self.N):
			# determine the desried commands
			
			# calculate the control

			# actuate the physical model

			# get the updated states ( meansurements )

			# update the history




	def plot(self):
		# plot the history to observe the output

		