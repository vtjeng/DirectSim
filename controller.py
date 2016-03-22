import numpy as np
from abc import ABCMeta, abstractmethod
import scipy
import logging

class AbstractController(object):
    __metaclass__ = ABCMeta

    def __init__(self, sensor, u_max):
        self.Sensor = sensor
        self.u_max = u_max

    @abstractmethod
    def compute_desired_u(self, raycast_distances):
        return 0

    def compute_u(self, raycastDistance):
        return self.cap_u(self.compute_desired_u(raycastDistance))

    def cap_u(self, u_desired):
        """
        Ensures that the maximum control input is no larger than the maximum allowed.
        :param u_desired: Value of u that we want the controller input to be.
        :return: value of u_desired capped to u_max.
        """
        if u_desired > self.u_max:
            return self.u_max
        elif u_desired < -self.u_max:
            return -self.u_max
        else:
            return u_desired

class CubicObjectiveController(AbstractController):
    def compute_desired_u(self, raycast_distances):
        w = [-0.12933211, -0.1168573, -0.18053814, -0.29548549, -0.43497715, -0.64520728, -0.93109957, -1.14716128, -1.19673222, -0.62145319, 0.61387357, 1.24814345, 1.16823557, 0.89885198, 0.64761157, 0.430246, 0.28741517, 0.17543379, 0.10411869, 0.13474329]
        u_desired = scipy.special.cbrt(np.dot(raycast_distances-self.Sensor.rayLength, w[::-1]))

        return u_desired

class WeightedCubicController(AbstractController):
    def compute_desired_u(self, raycast_distances):
        w = [-4.24024293e-05, -3.77577800e-05, -5.34051110e-05, -7.41927314e-05, -9.60329416e-05, -1.23734453e-04, -1.52172207e-04, -1.68049979e-04, -1.53122692e-04, -7.81028953e-05, 7.78999539e-05, 1.60691575e-04, 1.69868410e-04, 1.49521025e-04, 1.23502801e-04, 9.54336362e-05, 7.24562315e-05, 5.20114028e-05, 3.56561843e-05, 4.38290345e-05]
        u_desired = np.dot(raycast_distances**3, w[::-1])

        return u_desired


class WeightedLinearController(AbstractController):
    def compute_desired_u(self, raycast_distances):
        w = [-0.03095658, -0.01949426, -0.02677003, -0.03462618, -0.04171019, -0.05536625, -0.07269059, -0.08088806, -0.07962131, -0.03693672, 0.03636901, 0.08357396, 0.08290121, 0.0689061, 0.05494826, 0.04255617, 0.03428646, 0.02521712, 0.0174142, 0.02976362]
        u_desired = np.dot(raycast_distances, w[::-1])

        return u_desired

class CountIntersectionsController(AbstractController):
    def compute_desired_u(self, raycast_distances):
        tol = 1e-3

        left_sensor_distances = raycast_distances[0:self.Sensor.numRays / 2]
        right_sensor_distances = raycast_distances[self.Sensor.numRays / 2:]

        num_left_intersections = np.size(np.where(left_sensor_distances < self.Sensor.rayLength - tol))
        num_right_intersections = np.size(np.where(right_sensor_distances < self.Sensor.rayLength - tol))

        if num_left_intersections < num_right_intersections:
            u_desired = self.u_max
        elif num_left_intersections > num_right_intersections:
            u_desired = -self.u_max
        else:
            u_desired = 0

        return u_desired


class CountInverseDistancesController(AbstractController):
    def compute_desired_u(self, raycast_distances):
        left_sensor_distances = raycast_distances[0:self.Sensor.numRays / 2]
        right_sensor_distances = raycast_distances[self.Sensor.numRays / 2:]

        left_inverse_sum = np.sum(left_sensor_distances**(-2.0))
        right_inverse_sum = np.sum(right_sensor_distances**(-2.0))


        if left_inverse_sum < right_inverse_sum:
            u_desired = self.u_max
        elif left_inverse_sum > right_inverse_sum:
            u_desired = -self.u_max
        else:
            u_desired = 0

        return u_desired


class LeftTurnController(AbstractController):
    """
    A simple controller that demonstrates what turning left all the time does.
    """
    def compute_desired_u(self, raycast_distances):
        return self.u_max


