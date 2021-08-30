import numpy as np

class Quadrotor_model():
  def __init__(self):

    # define state vector
    # 12 states of quadrotor
    self.state = np.zeros((12,1), dtype=float)

    # INITAL STATE
    self.state[4] = 10.0
    self.state[6] = 0.2
    self.state[8] = -0.5
    self.state[10] = -0.2

    # define physical parameters
    self.g = 9.81
    self.Ixx = 0.04 # TUNABLE
    self.Iyy = 0.04 # TUNABLE
    self.Izz = 0.08 # TUNABLE
    self.m = 1.56 # TUNABLE
    self.I = np.diag([self.Ixx, self.Iyy, self.Izz])
    self.l = 0.25

    self.J = 90.0e-6 # MOTOR INTERTIA
    self.omega1 = 0.0
    self.omega2 = 0.0
    self.omega3 = 0.0
    self.omega4 = 0.0
    self.omega = 0.0
    self.b = 20.0e-6
    self.d = 0.4e-6

    # max control input possible
    self.Tmax = 40 # maximum thrust that can be applied to motors # TUNABLE
    self.Mmax = 2 # maximum moment that can be applied to motors # TUNABLE
    self.u_max = np.array([self.Mmax, self.Mmax, self.Mmax, self.Tmax]) 
    
    # convenience
    self.Niters = 0   

  def __str__(self):
      s  = "Quadrotor state after {} iters:\n".format(self.Niters)
      s += "\tposition:     {}.T\n".format(self.state[[0,2,4]].T)
      s += "\torient  :   {}.T\n".format(self.state[[6,8,10]].T)
      s += "\tvel     :     {}.T\n".format(self.state[[1,3,5]].T)
      s += "\tang_vel : {}.T\n".format(self.state[[7,9,11]].T)
      return s

  def update_states(self, u_c, dt):
    # dt: time interval to estimate numerical solutions to differential eqtn
    # u: the control input
      self.Niters += 1
      # print("N in QUAD_MODEL is ", self.Niters)
    # update function
      # clamp out the control input
      # u > self.max control matrix, jaha true aaya vaha max control matrix ke value
      compare_mat = u_c > self.u_max
      self.updated_u = compare_mat * self.u_max + (1-compare_mat) * u_c

      ## involve kinematics
      # https://pythonnumericalmethods.berkeley.edu/notebooks/chapter22.03-The-Euler-Method.html
      # update the values of p_n, p_e, h
      # print("ORIENTATION MATRIX ISSS")
      # print(self.orientation)
      PHI = self.state[6]
      PHI_dot = self.state[7]

      THETA = self.state[8]
      THETA_dot = self.state[9]

      PSI = self.state[10]
      PSI_dot = self.state[11]

      X =self.state[0]
      X_dot = self.state[1]

      Y = self.state[2]
      Y_dot = self.state[3]

      Z = self.state[4]
      Z_dot = self.state[5]

      # print("PHI:",PHI,"THETA:" , THETA,"PSI:", PSI)
      X = X + X_dot * dt
      Y = Y + Y_dot * dt
      Z = Z + Z_dot * dt 

      # update the values of PHI, THETA , PSI
      PHI = PHI + PHI_dot * dt
      THETA = THETA + THETA_dot * dt
      PSI = PSI + PSI_dot * dt

      ## involve dynamics
      self.omega1 = (- self.updated_u[1] / 2 * self.b) + (self.updated_u[2] / 4 * self.d) + (self.updated_u[3] / 4 * self.b)

      self.omega2 = (self.updated_u[1] / 2 * self.b) + (self.updated_u[2] / 4 * self.d) + (self.updated_u[3] / 4 * self.b)

      self.omega3 = (- self.updated_u[1] / 2 * self.b) + (- self.updated_u[2] / 4 * self.d) + (self.updated_u[3] / 4 * self.b)

      self.omega4 = (self.updated_u[1] / 2 * self.b) + (- self.updated_u[2] / 4 * self.d) + (self.updated_u[3] / 4 * self.b)

      self.omega = self.omega4 + self.omega3 - self.omega2 - self.omega1
      #linear

      X_dot_dot = ( (np.cos(PHI) * np.sin(THETA) * np.cos(PSI)) + (np.sin(PHI) * np.sin(PSI)) ) * self.updated_u[3] / self.m
      Y_dot_dot = ( (np.cos(PHI) * np.sin(THETA) * np.sin(PSI)) - (np.sin(PHI) * np.cos(PSI)) ) * self.updated_u[3] / self.m
      Z_dot_dot = ( np.cos(PHI) * np.cos(THETA) * self.updated_u[3] / self.m) - self.g
      print("Z",Z_dot_dot)
      X_dot = X_dot + X_dot_dot * dt
      Y_dot = Y_dot + Y_dot_dot * dt
      Z_dot = Z_dot + Z_dot_dot * dt


      # abgular
      PHI_dot_dot = ( (THETA_dot * PSI_dot) 
                          * (self.Iyy - self.Izz) 
                                  - (self.J * THETA_dot * self.omega ) 
                                        +  (self.l * self.updated_u[0]) ) / self.Ixx

      THETA_dot_dot = ( (PHI_dot * PSI_dot) 
                          * (self.Iyy - self.Ixx) 
                                  - (self.J * PHI_dot * self.omega ) 
                                        +  (self.l * self.updated_u[1]) ) / self.Iyy

      PSI_dot_dot = ( (THETA_dot * PHI_dot) 
                          * (self.Ixx - self.Iyy)  
                                      +  (self.updated_u[2]) ) / self.Izz

      PHI_dot = PHI_dot + PHI_dot_dot * dt
      THETA_dot = THETA_dot + THETA_dot_dot * dt
      PSI_dot = PSI_dot + PSI_dot_dot * dt

      self.state[6] = PHI
      self.state[7] = PHI_dot

      self.state[8] = THETA
      self.state[9] = THETA_dot

      self.state[10] = PSI
      self.state[11] = PSI_dot

      self.state[0] = X
      self.state[1] = X_dot

      self.state[2] = Y
      self.state[3] = Y_dot

      self.state[4] = Z
      self.state[5] = Z_dot

      return self.updated_u




# quad = Quadrotor_model()

# z_ff = quad.m * quad.g 
# print("Z FFFFF", z_ff)

# Ts = 0.01
# Tf = 0.06
# N = int(Tf/Ts)
# print(quad)
# print("ITERATIONS START TEJUU")
# for i in range(N):
#     m1 = 10 if Ts*i < 1 else 0
#     quad.update_states([0.0, 0.0, 0.0, z_ff], Ts)
#     print("-------------------------------------------------------------")
#     print(quad)
#     print("------------------------------------------------------------------")