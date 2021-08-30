
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
        self.g = 9.81
        self.Ixx = 0.04 # TUNABLE
        self.Iyy = 0.04 # TUNABLE
        self.Izz = 0.08 # TUNABLE
        self.m = 1.56 # TUNABLE
        self.I = np.diag([self.Ixx, self.Iyy, self.Izz])
        self.l = 0.25

        self.J = 90.0e-6

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
    
    def inner_loop(self, commanded, current_state, omega, Ts):
        

        # should return a 4 element vector - u1, u2, u3, u4
        # u corresponds to thrust, moment about roll, pitch and yaw
        # Transform the system into the form of HK (14.4) and (14.5)
        
        PHI = current_state[6]
        THETA = current_state[8]
        PSI = current_state[10]
        Z = current_state[4]
        print("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ", Z)

        PHI_dot = current_state[7] 
        THETA_dot = current_state[9]
        PSI_dot = current_state[11]
        Z_dot = current_state[5]
        


        #############################################################       roll      ###################################################################
        # desired stuff
        des_PHI = commanded[0]
        des_PHIdot = 0.0  # self.dPHIdt.update(des_PHI, Ts)
        des_PHIddot = 0.0  # self.dPHIdt2.update(des_PHIdot, Ts)

        # gains
        k_PHI = 1.0
        lambda_PHI = 3.0

        # error
        e_PHI = des_PHI - PHI
        e_PHIdot = des_PHIdot - PHI_dot

        # sliding variables
        s_PHI = e_PHIdot + lambda_PHI * e_PHI

        # sign checking might be needed
        u1 = (self.Ixx / self.l ) * ( 

                    (-THETA_dot * PSI_dot * (self.Iyy - self.Izz) / self.Ixx) +
                    des_PHIddot + 
                    lambda_PHI * e_PHIdot + 
                    k_PHI * np.sign(s_PHI) + (self.J * THETA_dot * omega / self.Ixx)   

                    )


        ###############################################################      pitch     ######################################################################
        # desired stuff
        des_THETA = commanded[1]
        des_THETAdot = 0.0 # self.dTHETAdt.update(des_THETA, Ts)
        des_THETAddot = 0.0 # self.dTHETAdt2.update(des_THETAdot, Ts)

        # gains
        k_THETA = 1.0
        lambda_THETA = 3.0

        # error
        e_THETA = des_THETA - THETA
        e_THETAdot = des_THETAdot - THETA_dot

        # sliding variables
        s_THETA = e_THETAdot + lambda_THETA * e_THETA

        # sign checking might be needed
        u2 = (self.Iyy / self.l) * ( 
                    (- PHI_dot * PSI_dot * (self.Izz - self.Ixx) / self.Iyy) +
                    des_THETAddot + 
                    lambda_THETA * e_THETAdot + 
                    k_THETA * np.sign(s_THETA) - (self.J * PHI_dot * omega / self.Iyy)
                    ) 


        #############################################################    yaw    ##############################################################
        # desired stuff
        des_PSI = commanded[2]
        des_PSIdot = self.dPSIdt.update(des_PSI, Ts)
        des_PSIddot = self.dPSIdt2.update(des_PSIdot, Ts)

        # gains
        k_PSI = 1.0
        lambda_PSI = 3.0

        # error
        e_PSI = des_PSI - PSI
        e_PSIdot = des_PSIdot - PSI_dot

        # sliding variables
        s_PSI = e_PSIdot + lambda_PSI * e_PSI

        # sign checking might be needed
        u3 = self.Izz * ( 
                        (-PHI_dot * THETA_dot * (self.Ixx - self.Iyy) / self.Izz) +
                        des_PSIddot + 
                        lambda_PSI * e_PSIdot + 
                        k_PSI * np.sign(s_PSI) 
                        ) 
                        


        ###################################################################      Z      ###################################################################
        # desired stuff
        des_Z = commanded[3]
        des_Zdot = 0.0  #self.dZdt.update(des_Z, Ts)
        des_Zddot = 0.0  #self.dZdt2.update(des_Zdot, Ts)

        # gains
        k_z = 6.0
        lambda_z = 3.0

        # error
        e_Z = des_Z - Z
        # print("ERRRRRRRRRRRRORRRRRRRRR",e_Z)
        e_Zdot = des_Zdot - Z_dot

        # sliding variable
        s_Z = e_Zdot + lambda_z * e_Z

        ## sign checking!!
        u4 = self.m * ( self.g + des_Zddot + (lambda_z * e_Zdot) + k_z * np.sign(s_Z)) / (np.cos(PHI) * (np.cos(THETA)))
        # print("CONTROL SCRIPT ALTITUDEEE", u4)
        # print("mass x gravity", self.m * self.g)
        u = np.hstack(np.array([u1, u2, u3, u4]))
        s = np.hstack(np.array([s_PHI, s_THETA, s_PSI, s_Z]))
        e = np.hstack(np.array([e_PHI, e_THETA, e_PSI, e_Z]))
        e_dot = np.hstack(np.array([e_PHIdot, e_THETAdot, e_PSIdot, e_Zdot]))


        return u, s, e, e_dot

    def outer(self):
        pass

    def update(self, commanded, state, omega, Ts):
        cmd = commanded[[6,7,8,2]]
        u, s, e, e_dot = self.inner_loop(cmd, state, omega, Ts)

        # actuator commands
        return u, s, e, e_dot


 

# controller = Controller()

# commanded = np.array([0., 0.0, 0.0, 3.0 ])
# state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Ts = 0.01

# a, b = controller.update(commanded, state, 0.2, Ts)

# print("control input", a)
# print("setpoints given", b)
