import numpy as np
import scipy.integrate as integrate

class CarPlant(object):


    """
    Specifies the behaviour of a Dubins Car.
    """

    def __init__(self, controller, velocity=12):
        x = 0.0
        y = 0.0
        psi = 0.0 * np.pi / 180.0

        self.state = np.array([x, y, psi])

        self.v = velocity

        self.Controller = controller


    def dynamics(self, state, t, controlInput=None):

        if controlInput is not None:
            u = controlInput
        else:
            u = self.Controller.computeControlInput()

        dqdt = np.zeros_like(state)
        dqdt[0] = self.v*np.cos(state[2])
        dqdt[1] = self.v*np.sin(state[2])
        dqdt[2] = u # we are directly controlling yaw rate
    
        return dqdt

    def setCarState(self, x, y, theta):
        self.state = np.array([x, y, theta])

    def simulateOneStep(self, startTime=0.0, dt=0.05, controlInput=None):
        t = np.linspace(startTime, startTime+dt, 2)
        newState = integrate.odeint(self.dynamics, self.state, t, args=(controlInput,))
        self.state = newState[-1,:]
        return self.state