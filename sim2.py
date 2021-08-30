import numpy as np
import matplotlib.pyplot as plt

class Simulator():
    def __init__(self, quad, ctrl, cmdr):
        # define inputs
        self.quadrotor_model = quad
        self.controller = ctrl
        self.commander = cmdr

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
        print("No of iterations used :", self.N)
        # quadrotor initial state
        self.truth = np.zeros((12,1))
        self.truth[0:3] = self.quadrotor_model.state[[0,2,4]]
        self.truth[3:6] = self.quadrotor_model.state[[1,3,5]]
        self.truth[6:9] = self.quadrotor_model.state[[6,8,10]]
        self.truth[9:12] = self.quadrotor_model.state[[7,9,11]]

        # define history components
        self.history["u"] = np.zeros((4,self.N))
        self.history["commands"] = np.zeros((12, self.N))
        self.history["truth"] = np.zeros((12, self.N))
        self.history["s"] = np.zeros((4,self.N))
        self.history["e"] = np.zeros((4,self.N))
        self.history["e_dot"] = np.zeros((4,self.N))


        # Now simulation starts
        for i in range(self.N):
            ## determine the desried commands
            ###### setpoint position
            ###### setpoint_position = np.array([])
            ###### setpoint orientation
            ###### setpoint vel
            ###### setpoint omega

            setpoints = self.commander.get_command()

            # calculate the control
            u, s, e, e_dot = self.controller.update(setpoints, self.quadrotor_model.state, self.quadrotor_model.omega, self.Ts)

            # actuate the physical model
            updated_u = self.quadrotor_model.update_states(u, Ts)

            # get the updated states ( meansurements )
            self.truth[0:3] = self.quadrotor_model.state[[0,2,4]]
            self.truth[3:6] = self.quadrotor_model.state[[1,3,5]]
            self.truth[6:9] = self.quadrotor_model.state[[6,8,10]]
            self.truth[9:12] = self.quadrotor_model.state[[7,9,11]]

            # update the history
            print("----------------------------------------------------------------")
            print(i, "iterations")
            self.history["u"][:,i] = u
            print("u", u)
            self.history["s"][:,i] = s
            self.history["e"][:,i] = e
            self.history["e_dot"][:,i] = e_dot
                                 
            self.history["commands"][:,i] = setpoints.flatten()
            print("setpoints", setpoints.flatten())
            self.history["truth"][:,i] = self.truth.flatten()
            print("truth", self.truth.flatten()) 
            print("----------------------------------------------------------------")



    def plot(self):
        """
        Plot
        Create plot(s) of the evolution of the quadrotor state
        over the simulation horizon.
        """
        plt.ioff()
        fig = plt.figure(figsize=(12,10))
        fig.subplots_adjust(wspace=0.25)
        fig.suptitle('Vehicle State', fontsize=16)
        
        tvec = (np.arange(self.N))*self.Ts
        
        #
        # Position
        #
        
        # for convenience
        xpos = self.history['truth'][0, :]
        ypos = self.history['truth'][1, :]
        zpos = self.history['truth'][2, :]
        
        xcmd = self.history['commands'][0, :]
        ycmd = self.history['commands'][1, :]
        zcmd = self.history['commands'][2, :]
        
        ax = fig.add_subplot(6,2,1)
        ax.plot(tvec, xcmd, 'r-', label='command')
        ax.plot(tvec, xpos, 'b-', label='truth')
        ax.set_ylabel('x'); ax.grid()
        ax.legend()

        ax = fig.add_subplot(6,2,3)
        ax.plot(tvec, ycmd, 'r-', label='command')
        ax.plot(tvec, ypos, 'b-', label='truth')
        ax.set_ylabel('y'); ax.grid()
        
        ax = fig.add_subplot(6,2,5)
        ax.plot(tvec, zcmd, 'r-', label='command')
        ax.plot(tvec, zpos, 'b-', label='truth')
        ax.set_ylabel('z'); ax.grid()
        
        #
        # Velocity
        #
        
        # for convenience
        xvel = self.history['truth'][3, :]
        yvel = self.history['truth'][4, :]
        zvel = self.history['truth'][5, :]
        
        xcmd = self.history['commands'][3, :]
        ycmd = self.history['commands'][4, :]
        zcmd = self.history['commands'][5, :]
        
        ax = fig.add_subplot(6,2,2)
        # if not np.isnan(xcmd).any():
        ax.plot(tvec, xcmd, 'r-', label='command')
        ax.plot(tvec, xvel, 'b-', label='truth')
        ax.set_ylabel('vx')
        ax.grid()

        ax = fig.add_subplot(6,2,4)
        # if not np.isnan(ycmd).any():
        ax.plot(tvec, ycmd, 'r-', label='command')
        ax.plot(tvec, yvel, 'b-', label='truth')
        ax.set_ylabel('vy')
        ax.grid()
        
        ax = fig.add_subplot(6,2,6)
        # if not np.isnan(zcmd).any():
        ax.plot(tvec, zcmd, 'r-', label='command')
        ax.plot(tvec, zvel, 'b-', label='truth')
        ax.set_ylabel('vz'); ax.grid()
        
        #
        # Attitude
        #
        
        # for convenience
        ph = self.history['truth'][6, :]
        th = self.history['truth'][7, :]
        ps = self.history['truth'][8, :]
        
        phcmd = self.history['commands'][6, :]
        thcmd = self.history['commands'][7, :]
        pscmd = self.history['commands'][8, :]
        
        ax = fig.add_subplot(6,2,7)
        ax.plot(tvec, np.degrees(phcmd), 'r-', label='command')
        ax.plot(tvec, np.degrees(ph), 'b-', label='truth')
        ax.set_ylabel(r'$\phi$'); ax.grid()

        ax = fig.add_subplot(6,2,9)
        ax.plot(tvec, np.degrees(thcmd), 'r-', label='command')
        ax.plot(tvec, np.degrees(th), 'b-', label='truth')
        ax.set_ylabel(r'$\theta$'); ax.grid()
        
        ax = fig.add_subplot(6,2,11)
        ax.plot(tvec, np.degrees(pscmd), 'r-', label='command')
        ax.plot(tvec, np.degrees(ps), 'b-', label='truth')
        ax.set_ylabel(r'$\psi$'); ax.grid()
        
        #
        # Angular Rates
        #
        
        # for convenience
        p = self.history['truth'][9, :]
        q = self.history['truth'][10, :]
        r = self.history['truth'][11, :]

        pcmd = self.history['commands'][9, :]
        qcmd = self.history['commands'][10, :]
        rcmd = self.history['commands'][11, :]
        
        ax = fig.add_subplot(6,2,8)
        ax.plot(tvec, pcmd, 'r-', label='command')
        ax.plot(tvec, p, 'b-', label='truth')
        ax.set_ylabel('p'); ax.grid()

        ax = fig.add_subplot(6,2,10)
        if not np.isnan(qcmd).any():
            ax.plot(tvec, qcmd, 'r-', label='command')
        ax.plot(tvec, q, 'b-', label='truth')
        ax.set_ylabel('q'); ax.grid()
        
        ax = fig.add_subplot(6,2,12)
        ax.plot(tvec, rcmd, 'r-', label='command')
        ax.plot(tvec, r, 'b-', label='truth')
        ax.set_ylabel('r'); ax.grid()
        
        # plt.show()
        
        #
        # Control Effort
        #
        
        tau_ph = self.history['u'][0, :]
        tau_th = self.history['u'][1, :]
        tau_ps = self.history['u'][2, :]
        thrust = self.history['u'][3, :]
        
        fig2 = plt.figure(figsize=(12,3))
        fig2.subplots_adjust(wspace=0.25)
        fig2.suptitle('Control Effort', fontsize=16)
        
        ax = fig2.add_subplot(2,2,1)
        ax.plot(tvec, thrust, 'g-')
        ax.set_ylabel('Thrust'); ax.grid()
        
        ax = fig2.add_subplot(2,2,2)
        ax.plot(tvec, tau_ph, 'g-')
        ax.set_ylabel(r'$\tau_\phi$'); ax.grid()
        
        ax = fig2.add_subplot(2,2,3)
        ax.plot(tvec, tau_th, 'g-')
        ax.set_ylabel(r'$\tau_\theta$'); ax.grid()
        
        ax = fig2.add_subplot(2,2,4)
        ax.plot(tvec, tau_ps, 'g-')
        ax.set_ylabel(r'$\tau_\psi$'); ax.grid()
        
        

        ###############
        s_PHI_ = self.history['s'][0, :]
        s_THETA_ = self.history['s'][1, :]
        s_PSI_ = self.history['s'][2, :]
        s_Z_ = self.history['s'][3, :]
        
        fig3 = plt.figure(figsize=(12,3))
        fig3.subplots_adjust(wspace=0.25)
        fig3.suptitle('Sliding Variable', fontsize=16)
        
        ax = fig3.add_subplot(2,2,1)
        ax.plot(tvec, s_PHI_, 'g-')
        ax.set_ylabel('S_PHI'); ax.grid()
        
        ax = fig3.add_subplot(2,2,2)
        ax.plot(tvec, s_THETA_, 'g-')
        ax.set_ylabel("S_THETA"); ax.grid()
        
        ax = fig3.add_subplot(2,2,3)
        ax.plot(tvec, s_PSI_, 'g-')
        ax.set_ylabel("S_PSI"); ax.grid()
        
        ax = fig3.add_subplot(2,2,4)
        ax.plot(tvec, s_Z_, 'g-')
        ax.set_ylabel("S_Z"); ax.grid()
        

        ##############

        e_PHI_ = self.history['e'][0, :]
        e_THETA_ = self.history['e'][1, :]
        e_PSI_ = self.history['e'][2, :]
        e_Z_ = self.history['e'][3, :]
        
        fig4 = plt.figure(figsize=(12,3))
        fig4.subplots_adjust(wspace=0.25)
        fig4.suptitle('ERRROR', fontsize=16)
        
        ax = fig4.add_subplot(2,2,1)
        ax.plot(tvec, e_PHI_, 'g-')
        ax.set_ylabel('E_PHI_'); ax.grid()
        
        ax = fig4.add_subplot(2,2,2)
        ax.plot(tvec, e_THETA_, 'g-')
        ax.set_ylabel("e_THETA_"); ax.grid()
        
        ax = fig4.add_subplot(2,2,3)
        ax.plot(tvec, e_PSI_, 'g-')
        ax.set_ylabel("E_PSI"); ax.grid()
        
        ax = fig4.add_subplot(2,2,4)
        ax.plot(tvec, e_Z_, 'g-')
        ax.set_ylabel("E_Z"); ax.grid()
        

        ##############

        e_dot_PHI_ = self.history['e_dot'][0, :]
        e_dot_THETA_ = self.history['e_dot'][1, :]
        e_dot_PSI_ = self.history['e_dot'][2, :]
        e_dot_Z_ = self.history['e_dot'][3, :]
        
        fig5 = plt.figure(figsize=(12,3))
        fig5.subplots_adjust(wspace=0.25)
        fig5.suptitle('ERROR DERIVATIVE', fontsize=16)
        
        ax = fig5.add_subplot(2,2,1)
        ax.plot(tvec, e_dot_PHI_, 'g-')
        ax.set_ylabel('e_dot_PHI_'); ax.grid()
        
        ax = fig5.add_subplot(2,2,2)
        ax.plot(tvec, e_dot_THETA_, 'g-')
        ax.set_ylabel("e_dot_THETA_"); ax.grid()
        
        ax = fig5.add_subplot(2,2,3)
        ax.plot(tvec, e_dot_PSI_, 'g-')
        ax.set_ylabel("e_dot_PSI_"); ax.grid()
        
        ax = fig5.add_subplot(2,2,4)
        ax.plot(tvec, e_dot_Z_, 'g-')
        ax.set_ylabel("e_dot_Z_"); ax.grid()
        
        plt.show()

        



 