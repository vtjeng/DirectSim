import director.vtkAll as vtk

import numpy as np


class SensorObj(object):

    def __init__(self, num_rays=20, ray_length=8, fov=180.0):
        self.numRays = num_rays
        self.rayLength = ray_length

        FOVrad = fov * np.pi / 180.0
        self.angleMin = -FOVrad/2
        self.angleMax = FOVrad/2

        self.angleGrid = np.linspace(self.angleMin, self.angleMax, self.numRays)

        self.rays = np.zeros((3,self.numRays))
        self.rays[0,:] = np.cos(self.angleGrid)
        self.rays[1,:] = -np.sin(self.angleGrid)

    # TODO: ask pete what a locator is.
    def setLocator(self, locator):
        self.locator = locator

    def raycast_all(self, frame):

        distances = np.zeros(self.numRays)

        origin = np.array(frame.transform.GetPosition())

        for i in range(0,self.numRays):
            ray = self.rays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            intersection = self.raycast(self.locator, origin, origin + rayTransformed*self.rayLength)
            if intersection is None:
                distances[i] = self.rayLength
            else:
                distances[i] = np.linalg.norm(intersection - origin)

        return distances

    def raycast(self, locator, ray_origin, ray_end):

        tolerance = 0.0 # intersection tolerance
        pt = [0.0, 0.0, 0.0] # data coordinate where intersection occurs
        lineT = vtk.mutable(0.0) # parametric distance along line segment where intersection occurs
        pcoords = [0.0, 0.0, 0.0] # parametric location within cell (triangle) where intersection occurs
        subId = vtk.mutable(0) # sub id of cell intersection

        result = locator.IntersectWithLine(ray_origin, ray_end, tolerance, lineT, pt, pcoords, subId)

        return pt if result else None