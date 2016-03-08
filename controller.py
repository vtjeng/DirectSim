import numpy as np
from abc import ABCMeta, abstractmethod


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
        :param u_desired:
        :return: value of u capped to u_max.
        """
        if u_desired > self.u_max:
            return self.u_max
        elif u_desired < -self.u_max:
            return -self.u_max
        else:
            return u_desired


class SupervisedCubicController(AbstractController):
    def compute_desired_u(self, raycast_distances):
        w = np.array([-0.00035671, -0.00035193, -0.00053087, -0.00084506, -0.00121055, -0.00183538, -0.0026913, -0.00332851, -0.00267597, -0.00112497, 0.00112066, 0.00260913, 0.00343951, 0.00269573, 0.001834, 0.00120245, 0.00089366, 0.00056788, 0.00036757, 0.00038036])
        u_desired = np.dot(raycast_distances**3, w[::-1])

        return u_desired


class SupervisedLinearController(AbstractController):
    def compute_desired_u(self, raycast_distances):
        w = np.array([-0.01457873, -0.01027499, -0.01612163, -0.02389832, -0.03228708, -0.04974498, -0.07291344, -0.09525306, -0.07906981, -0.03315874, 0.03480734, 0.07656544, 0.09687568, 0.07201174, 0.04925396, 0.03240909, 0.026072, 0.0173852, 0.01189639, 0.01622941])
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


