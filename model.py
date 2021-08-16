import numpy as np



class Quadrotor_model():
	def __init__(self):

		# define state vector
		# 12 states of quadrotor
		self.position = np.zeros(3,1) # defined in inertial frame #p_n, p_e, h
		self.vel = np.zeros(3,1) # defined in body frame # u, v, w
		self.orien = np.zeros(3,1) # defined roll,pitch and yaw angle wrt v,v1,v2 # phi, theta, psi
		self.angular_vel = np.zeros(3,1) # defined roll, pitch and yaw rates in body frame # p,q,r


		# define physical parameters
		self.g = 9.81
		Jxx = None
		Jyy = None
		Jzz = None
		self.I = np.diag(Jxx, Jyy, Jzz)

		# max control input possible
		self.Tmax = None # maximum thrust that can be applied to motors
		self.Mmax = None # maximum moment that can be applied to motors

		update_states(self, dt, u):
		# dt: time interval to estimate numerical solutions to differential eqtn
		# u: the control input
		
		# update function
			# involve kinematics
				# update the values of p_n, p_e, h
				# update the values
			# clamp out control input
			# involve dynamics
			# give out the updated control input










