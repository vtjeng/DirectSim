import numpy as np
import scipy.integrate as integrate
import director.objectmodel as om


class ControllerObj(object):

    def __init__(self, sensor, u_max=1, epsilonRand=0.4):
        self.Sensor = sensor
        self.u_max = u_max
        self.actionSet = np.array([u_max,0,-u_max])
        self.epsilonRand = epsilonRand
        self.actionSetIdx = np.arange(0,np.size(self.actionSet))

        # TODO: split this controller up?

    def computeControlInput(self, state, t, frame, raycastDistance=None, randomize=False):
        self.distances = raycastDistance

        u, actionIdx = self.supervisedDPControllerCubic()

        # TODO: Ask PETE - What does the randomization here achieve?
        if randomize:
            if np.random.uniform(0,1,1)[0] < self.epsilonRand:
                # otherActionIdx = np.setdiff1d(self.actionSetIdx, np.array([actionIdx]))
                # randActionIdx = np.random.choice(otherActionIdx)
                actionIdx = np.random.choice(self.actionSetIdx)
                u = self.actionSet[actionIdx]

        # TODO: Remove indexes if they aren't necessary
        return u, actionIdx


    def countStuffController(self, raycastDistance):
        firstHalf = raycastDistance[0:self.Sensor.numRays/2]
        secondHalf = raycastDistance[self.Sensor.numRays/2:]
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

    def countInverseDistancesController(self, raycastDistance):
        midpoint = np.floor(self.Sensor.numRays/2.0)
        leftHalf = np.array((raycastDistance[0:midpoint]))
        rightHalf = np.array((raycastDistance[midpoint:]))
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

        u = self.actionSet[actionIdx]
        return u, actionIdx

    def supervisedDPController(self):

        #9 sensors
        #w = np.array([-0.03364086, -0.06146491, -0.11796771, -0.1699006,  -0.00097573,  0.17137526, 0.11952639,  0.06076998,  0.03276566])

        # 20 sensors slow
        #w = [-0.02109653, -0.01746332, -0.02388135, -0.0314405,  -0.04294771, -0.05559809, -0.07757404, -0.08611176, -0.07874338, -0.04490507,  0.04384566,  0.08218653, 0.08214135,  0.08184778,  0.05594081,  0.04173576,  0.03131204,  0.02372157, 0.01681253,  0.02070505]
        
        # we're reversing the w array because of the vagaries of the ML.
        w = np.array([-0.01457873, -0.01027499, -0.01612163, -0.02389832, -0.03228708, -0.04974498, -0.07291344, -0.09525306, -0.07906981, -0.03315874, 0.03480734, 0.07656544, 0.09687568, 0.07201174, 0.04925396, 0.03240909, 0.026072, 0.0173852, 0.01189639, 0.01622941])

        u_desired = np.dot(self.distances, w[::-1])

        if u_desired > self.u_max:
            return self.u_max, 0
        elif u_desired < -self.u_max:
            return -self.u_max, 0
        else:
            return u_desired, 0

    def supervisedDPControllerCubic(self):
        # we're reversing the w array because of the vagaries of the ML.
        w = np.array([-0.00035671, -0.00035193, -0.00053087, -0.00084506, -0.00121055, -0.00183538, -0.0026913, -0.00332851, -0.00267597, -0.00112497, 0.00112066, 0.00260913, 0.00343951, 0.00269573, 0.001834, 0.00120245, 0.00089366, 0.00056788, 0.00036757, 0.00038036])

        u_desired = np.dot(self.distances**3, w[::-1])

        if u_desired > self.u_max:
            return self.u_max, 0
        elif u_desired < -self.u_max:
            return -self.u_max, 0
        else:
            return u_desired, 0


    def computeControlInputFromFrame(self):
        carState = 0
        t = 0
        frame = om.findObjectByName('robot frame')
        return self.computeControlInput(carState, t, frame, self.Sensor.raycastAll(frame))

