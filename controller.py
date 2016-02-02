import numpy as np
import scipy.integrate as integrate
import ddapp.objectmodel as om


class ControllerObj(object):

    def __init__(self, sensor, u_max=4, epsilonRand=0.4):
        self.Sensor = sensor
        self.numRays = self.Sensor.numRays
        self.actionSet = np.array([u_max,0,-u_max])
        self.epsilonRand = epsilonRand
        self.actionSetIdx = np.arange(0,np.size(self.actionSet))

    def computeControlInput(self, state, t, frame, raycastDistance=None, randomize=False):
        # test cases
        # u = 0
        # u = np.sin(t)
        if raycastDistance is None:
            self.distances = self.Sensor.raycastAll(frame)
        else:
            self.distances = raycastDistance

        # #Barry 12 controller
        

        #u = self.countStuffController()
        u, actionIdx = self.countInverseDistancesController()
        u, actionIdx = self.supervisedDPController()

        if randomize:
            if np.random.uniform(0,1,1)[0] < self.epsilonRand:
                # otherActionIdx = np.setdiff1d(self.actionSetIdx, np.array([actionIdx]))
                # randActionIdx = np.random.choice(otherActionIdx)
                actionIdx = np.random.choice(self.actionSetIdx)
                u = self.actionSet[actionIdx]

        return u, actionIdx


    def countStuffController(self):
        firstHalf = self.distances[0:self.numRays/2]
        secondHalf = self.distances[self.numRays/2:]
        tol = 1e-3;

        numLeft = np.size(np.where(firstHalf < self.Sensor.rayLength - tol))
        numRight = np.size(np.where(secondHalf < self.Sensor.rayLength - tol))

        if numLeft == numRight:
            actionIdx = 1
        elif numLeft > numRight:
            actionIdx = 2
        else:
            actionIdx = 0

        u = self.actionSet[actionIdx]
        return u, actionIdx

    def countInverseDistancesController(self):
        midpoint = np.floor(self.numRays/2.0)
        leftHalf = np.array((self.distances[0:midpoint]))
        rightHalf = np.array((self.distances[midpoint:]))
        tol = 1e-3;

        inverseLeftHalf = (1.0/leftHalf)**2
        inverseRightHalf = (1.0/rightHalf)**2

        numLeft = np.sum(inverseLeftHalf)
        numRight = np.sum(inverseRightHalf)


        if numLeft == numRight:
            actionIdx = 1
        elif numLeft > numRight:
            actionIdx = 2
        else:
            actionIdx = 0


        # print "leftHalf ", leftHalf
        # print "rightHalf", rightHalf
        # print "inverseLeftHalf", inverseLeftHalf
        # print "inverserRightHalf", inverseRightHalf
        # print "numLeft", numLeft
        # print "numRight", numRight

        u = self.actionSet[actionIdx]
        return u, actionIdx

    def supervisedDPController(self):

        #9 sensors
        #w = np.array([-0.03364086, -0.06146491, -0.11796771, -0.1699006,  -0.00097573,  0.17137526, 0.11952639,  0.06076998,  0.03276566])

        # 20 sensors slow
        #w = [-0.02109653, -0.01746332, -0.02388135, -0.0314405,  -0.04294771, -0.05559809, -0.07757404, -0.08611176, -0.07874338, -0.04490507,  0.04384566,  0.08218653, 0.08214135,  0.08184778,  0.05594081,  0.04173576,  0.03131204,  0.02372157, 0.01681253,  0.02070505]
        

        w = np.array([-0.00300497, -0.00130277, -0.00148445, -0.00313336, -0.01317847, -0.02037713, -0.04797057, -0.09098885, -0.13847444, -0.11547472,  0.11733177,  0.13888244, 0.08363806,  0.04846861,  0.02326903,  0.01233246,  0.00382634,  0.00258145, 0.00284502,  0.00306195])
        w = w[::-1]

        u = np.dot(self.distances, w)

        #if u > 4: u = 4
        #if u < -4: u = -4

        return u, 0


    def computeControlInputFromFrame(self):
        carState = 0
        t = 0
        frame = om.findObjectByName('robot frame')
        return self.computeControlInput(carState, t, frame)

