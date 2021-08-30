
from derivative_lpf import DirtyDerivative
import numpy as np
## code requires restructuring , many things written explictly could be
# written with matrix algebra

class Controller(object):
    """Controller
    """
    def __init__(self):
        self.name = "CLASSICAL SMC"

        # estimates of physical properties of the quadrotor
        self.g = 9.8
        # self.m = 4
        # Jxx = 0.1; Jyy = 0.1; Jzz = 0.1
        Jxx = 0.060224
        Jyy = 0.122198
        Jzz = 0.132166
        self.m = 3.81
        self.I = np.array([[Jxx,0,0],
                           [0,Jyy,0],
                           [0,0,Jzz]])

        # dirty derivative filters
        self.dZdt = DirtyDerivative(order=1, tau=0.05)
        self.dZdt2 = DirtyDerivative(order=2, tau=0.05)
        self.dPHIdt = DirtyDerivative(order=1, tau=0.05)
        self.dPHIdt2 = DirtyDerivative(order=2, tau=0.05)
        self.dTHETAdt = DirtyDerivative(order=1, tau=0.05)
        self.dTHETAdt2 = DirtyDerivative(order=2, tau=0.05)
        self.dPSIdt = DirtyDerivative(order=1, tau=0.05)
        self.dPSIdt2 = DirtyDerivative(order=2, tau=0.05)
    
    def __str__(self):
        return self.name
    
    def inner_loop(self, commanded, current_state, Ts):
        

        # should return a 4 element vector - u1, u2, u3, u4
        # u corresponds to thrust, moment about roll, pitch and yaw
        # Transform the system into the form of HK (14.4) and (14.5)
        req_states = np.array([current_state.flatten()[[2, 6, 7, 8]]]).T # r_z, ph, th, ps
        req_states_d = np.array([current_state.flatten()[[5, 9, 10, 11]]]).T # \dot{r_z}, p, q, r
        
        phi = req_states.flatten()[1]
        theta = req_states.flatten()[2]
        psi = req_states.flatten()[3]
        
        Jx = self.I[0,0]
        Jy = self.I[1,1]
        Jz = self.I[2,2]

        ###### r_z ######
        # desired stuff
        des_Z = commanded[0]
        des_Zdot = self.dZdt.update(des_Z, Ts)
        des_Zddot = self.dZdt2.update(des_Zdot, Ts)

        # gains
        k_z = 6.0
        lambda_z = 3.0

        # error
        e_Z = des_Z - req_states[0]
        e_Zdot = des_Zdot - req_states_d[0]

        # sliding variable
        s_Z = e_Zdot + lambda_z * e_Z

        ## sign checking!!
        u1 = self.m * ( self.g + des_Zddot - (lambda_z * e_Zdot) + k_z * np.sign( s_Z)) / (np.cos(phi) * (np.cos(theta)))

        ##### roll ###
        # desired stuff
        des_PHI = commanded[1]
        des_PHIdot = self.dPHIdt.update(des_PHI, Ts)
        des_PHIddot = self.dPHIdt2.update(des_PHIdot, Ts)

        # gains
        k_PHI = 1.0
        lambda_PHI = 3.0

        # error
        e_PHI = des_PHI - req_states[1]
        e_PHIdot = des_PHIdot - req_states_d[1]

        # sliding variables
        s_PHI = e_PHIdot + lambda_PHI * e_PHI

        # sign checking might be needed
        u2 = Jx * ( (req_states_d[2] * req_states_d[3] * (Jz - Jy) / Jx) +
                    des_PHIddot + 
                    lambda_PHI * e_PHIdot + 
                    k_PHI * np.sign(s_PHI) )   


        ##### pitch ###
        # desired stuff
        des_THETA = commanded[2]
        des_THETAdot = self.dTHETAdt.update(des_THETA, Ts)
        des_THETAddot = self.dTHETAdt2.update(des_THETAdot, Ts)

        # gains
        k_THETA = 1.0
        lambda_THETA = 3.0

        # error
        e_THETA = des_THETA - req_states[2]
        e_THETAdot = des_THETAdot - req_states_d[2]

        # sliding variables
        s_THETA = e_THETAdot + lambda_THETA * e_THETA

        # sign checking might be needed
        u3 = Jx * ( (req_states_d[1] * req_states_d[3] * (Jx - Jz) / Jy) +
                    des_THETAddot + 
                    lambda_THETA * e_THETAdot + 
                    k_THETA * np.sign(s_THETA) ) 


        ##### yaw ###
        # desired stuff
        des_PSI = commanded[3]
        des_PSIdot = self.dPSIdt.update(des_PSI, Ts)
        des_PSIddot = self.dPSIdt2.update(des_PSIdot, Ts)

        # gains
        k_PSI = 1.0
        lambda_PSI = 3.0

        # error
        e_PSI = des_PSI - req_states[3]
        e_PSIdot = des_PSIdot - req_states_d[3]

        # sliding variables
        s_PSI = e_PSIdot + lambda_PSI * e_PSI

        # sign checking might be needed
        u4 = Jx * ( (req_states_d[1] * req_states_d[2] * (Jy - Jx) / Jz) +
                    des_PSIddot + 
                    lambda_PSI * e_PSIdot + 
                    k_PSI * np.sign(s_PSI) ) 

        u = np.hstack(np.array([u1, u2, u3, u4]))

        return u

    def outer(self):
        pass

    def update(self, commanded, state, Ts):
        cmd = commanded[[2,6,7,8]]
        u = self.inner_loop(cmd, state, Ts)

        # actuator commands
        return u, commanded


 

# controller = Controller()

# commanded = np.array([0., 0., -2., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
# state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Ts = 0.01

# a, b = controller.update(commanded, state, Ts)

# print("control input", a)
# print("setpoints given", b)
