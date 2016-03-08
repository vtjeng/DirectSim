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

    def dynamics(self, state, t, control_input=None):
        if control_input is not None:
            u = control_input
        else:
            u = self.Controller.compute_u()

        dqdt = np.zeros_like(state)
        dqdt[0] = self.v*np.cos(state[2])
        dqdt[1] = self.v*np.sin(state[2])
        dqdt[2] = u # we are directly controlling yaw rate
    
        return dqdt

    def set_car_state(self, x, y, theta):
        self.state = np.array([x, y, theta])

    def simulate_one_step(self, start_time=0.0, dt=0.05, control_input=None):
        """
        Simulates the dynamics of the car, modifying the state of the car and returning the new state.
        :param start_time: Time at which we are starting the simulation
        :param dt: Timestep
        :param control_input: Control input to vehicle.
        :return: New state of car.
        """
        t = np.linspace(start_time, start_time + dt, 2)
        newState = integrate.odeint(self.dynamics, self.state, t, args=(control_input,))
        self.state = newState[-1,:]
        return self.state