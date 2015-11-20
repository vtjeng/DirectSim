__author__ = 'manuelli'
import utils
import ddapp.objectmodel as om
import numpy as np


class Reward(object):

    def __init__(self, sensorObj=None, collisionThreshold=None, collisionPenalty=300.0,
                 actionCost=1.0):
        if sensorObj is None or collisionThreshold is None:
            ValueError("need to specify sensorObj, actionSet and collisionThreshold")
        self.numRays = sensorObj.numRays
        self.rayLength = sensorObj.rayLength
        self.sensorObj = sensorObj
        self.collisionThreshold = collisionThreshold
        self.collisionPenalty = collisionPenalty
        self.actionCost = actionCost
        self.initializeRaycastRewardWeights()
        self.largeConstant = 1e5*1.0
        self.tol = 1e-3
        self.cutoff = 20

    def initializeRaycastRewardWeights(self):
        self.raycastRewardWeights = -5*np.ones(self.numRays)

    def checkInCollision(self, raycastDistance):
        if np.min(raycastDistance) < self.collisionThreshold:
            return True
        else:
            return False

    def computeReward(self, S, u):
        carState, raycastDistance = S
        if self.checkInCollision(raycastDistance):
            return -self.collisionPenalty

        reward = -self.actionCost*np.linalg.norm(u)
        reward += self.computeRaycastReward(S, u)
        return reward

    def computeRaycastReward(self, S, u):
        carState, raycastDistance = S
        raycastAdjusted = raycastDistance - self.collisionThreshold
        maxRangeIdx = np.where(raycastDistance > self.rayLength - self.tol)
        raycastAdjusted[maxRangeIdx] = self.largeConstant

        inverseTruncated = utils.inverseTruncate(raycastAdjusted, self.cutoff)
        return np.dot(self.raycastRewardWeights, inverseTruncated)

    def computeRewardFromFrameLocation(self):
        carFrame = om.findObjectByName('robot frame')
        raycastDistance = self.sensorObj.raycastAll(carFrame)

        u = 0.0;
        carState = 0.0 # just a placeholder for now
        S = (carState, raycastDistance)
        return self.computeReward(S, u)






