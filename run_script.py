from command import commander
from control2 import Controller
from derivative_lpf import DirtyDerivative
from model2 import Quadrotor_model
from sim2 import Simulator

# Instantiate a quadrotor model with the given initial conditions
quad = Quadrotor_model()

# Instantiate the SMC controller
ctrl = Controller()

# Setup a setpoint commander
cmdr = commander()

# Run the simulation
sim = Simulator(quad, ctrl, cmdr)
sim.run(20, Ts=0.01)
sim.plot()
