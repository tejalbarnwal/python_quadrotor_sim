from command import commander
from control import Controller
from derivative_lpf import DirtyDerivative
from model import Quadrotor_model
from sim import Simulator

# Instantiate a quadrotor model with the given initial conditions
quad = Quadrotor_model()

# Instantiate the SMC controller
ctrl = Controller()

# Setup a setpoint commander
cmdr = commander()

# Run the simulation
sim = Simulator(quad, ctrl, cmdr)
sim.run(200, Ts=0.01)
sim.plot()
