import numpy as np

def Rot_v_to_v1(psi):
    R = np.array([
        [ np.cos(psi), np.sin(psi), 0],
        [-np.sin(psi), np.cos(psi), 0],
        [     0,           0    ,   1]
    ])
    return R

def Rot_v1_to_v2(theta):
    R = np.array([
        [np.cos(theta), 0, -np.sin(theta)],
        [      0      , 1,        0      ],
        [np.sin(theta), 0,  np.cos(theta)]
    ])
    return R

def Rot_v2_to_b(phi):
    # R = np.array([  [1,       0     ,      0     ],
    #         [0,  np.cos(phi), np.sin(phi)],
    #           [0, -1 * np.sin(phi), np.cos(phi)]  
    #         ])
    R = np.array([
        [1,0,0],
        [0,np.cos(phi),np.sin(phi)],
        [0,-np.sin(phi),np.cos(phi)]
      ])
    return R

def Rot_v_to_b(phi, theta, psi):
    return np.dot( np.dot(Rot_v2_to_b(phi),Rot_v1_to_v2(theta)) , Rot_v_to_v1(psi) )

def Rot_i_to_b(phi, theta, psi):
    return Rot_v_to_b(phi, theta, psi)

def Rot_angVel_to_angRates(phi, theta, psi):
  R = np.array([  [1,     np.sin(phi)*np.tan(theta),    np.cos(phi)*np.tan(theta) ],
          [0,     np.cos(phi),                  -np.sin(phi)              ],
          [0,     np.sin(phi)/np.cos(theta),    np.cos(phi)/np.cos(theta) ]])
  return R

    

class Quadrotor_model():
  def __init__(self):

    # define state vector
    # 12 states of quadrotor
    self.position = np.zeros((3,1),dtype=float) # defined in inertial frame #p_n, p_e, h
    self.vel = np.zeros((3,1),dtype=float) # defined in body frame # u, v, w
    self.orientation = np.zeros((3,1),dtype=float) # defined roll,pitch and yaw angle wrt v,v1,v2 # phi, theta, psi
    self.angular_vel = np.zeros((3,1),dtype=float) # defined roll, pitch and yaw rates in body frame # p,q,r


    # define physical parameters
    self.g = 9.81
    self.Jxx = 0.060224 # TUNABLE
    self.Jyy = 0.122198 # TUNABLE
    self.Jzz = 0.132166 # TUNABLE
    self.m = 3.81 # TUNABLE
    self.I = np.diag([self.Jxx, self.Jyy, self.Jzz])

    # max control input possible
    self.Tmax = 40 # maximum thrust that can be applied to motors # TUNABLE
    self.Mmax = 2 # maximum moment that can be applied to motors # TUNABLE
    self.u_max = np.array([self.Tmax, self.Mmax, self.Mmax, self.Mmax]) 
    
    # convenience
    self.Niters = 0   

  def __str__(self):
      s  = "Quadrotor state after {} iters:\n".format(self.Niters)
      s += "\tposition:     {}.T\n".format(self.position.T)
      s += "\torient  :   {}.T\n".format(self.orientation.T)
      s += "\tvel     :     {}.T\n".format(self.vel.T)
      s += "\tang_vel : {}.T\n".format(self.angular_vel.T)
      return s

  def update_states(self, u_c, dt):
    # dt: time interval to estimate numerical solutions to differential eqtn
    # u: the control input
      self.Niters += 1
      print("N in QUAD_MODEL is ", self.Niters)
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
      phi, theta, psi = self.orientation.flatten()
      # print("phi:",phi,"theta:" , theta,"psi:", psi)
      self.position = self.position + np.dot(Rot_i_to_b(phi, theta, psi).transpose(), self.vel) * dt 
      # update the values of phi, theta , psi
      self.orientation = self.orientation + np.dot(Rot_angVel_to_angRates(phi, theta, psi),self.angular_vel) * dt

      ## involve dynamics
      #linear
      p,q,r = self.angular_vel.flatten()
      u, v, w = self.vel.flatten()

      self.gravity_matrix = np.array([[-self.g * np.sin(theta)]                , 
                      [self.g * np.cos(theta) * np.sin(theta)] ,
                      [self.g * np.cos(theta) * np.cos(phi)]   ])

      self.lin_crossfactor = np.array([[r*v-q*w], [p*w-r*u], [q*u-p*v]])

      self.lin_cntrl = np.array([[0], [0], [-self.updated_u[0] ] ])
      
      self.vel = self.vel + ( self.lin_crossfactor + self.gravity_matrix + self.lin_cntrl * (1.0/self.m) ) * dt

      # abgular
      self.rot_crossfactor = np.array([[ (self.Jyy - self.Jxx) * q * r / self.Jxx],
                        [ (self.Jzz - self.Jxx) * p * r / self.Jyy],
                        [ (self.Jxx - self.Jyy) * p * q / self.Jzz] ])

      self.dynamic_cntrl = np.array([ [ self.updated_u[1]/self.Jxx] ,
                      [ self.updated_u[2]/self.Jyy] ,
                      [ self.updated_u[3]/self.Jzz]  ])

      self.angular_vel = self.angular_vel + (self.rot_crossfactor + self.dynamic_cntrl) * dt



# quad = Quadrotor_model()

# z_ff = quad.m*quad.g*10

# Ts = 0.01
# Tf = 2
# N = int(Tf/Ts)
# for i in range(N):
#     m1 = 10 if Ts*i < 1 else 0
#     quad.update_states([z_ff, 0.0, 0.0, 0.0], Ts)
# print(quad)







