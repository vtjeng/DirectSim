import argparse
import numpy as np
import shelve
import time

import director.objectmodel as om
import director.visualization as vis
import director.vtkAll as vtk
from PythonQt import QtCore, QtGui
from director import applogic
from director import cameracontrolpanel
from director import screengrabberpanel
from director.consoleapp import ConsoleApp
from director.debugVis import DebugData
from director.timercallback import TimerCallback

from carPlant import CarPlant
from controller import *
from sensor import SensorObj
from world import World
from bunch import *

import logging

class Simulator(object):


    def __init__(self, endTime=40, options = None):

        self.logger = Simulator.initialize_loggers() # TODO: give user an option to turn off logging

        self.startSimTime = time.time()
        self.collisionThreshold = 0.7

        self.logger.info('Collision threshold is {}'.format(self.collisionThreshold))

        # create the visualizer object
        self.app = ConsoleApp()
        self.view = self.app.createView(useGrid=False)

        if options != None:
            self.options = options
        else:
            self.options = Simulator.getDefaultOptions()

        self.logger.info('Options are {}'.format(self.options.toDict()))

        self.initializeColorMap()

    @staticmethod
    def initialize_loggers():

        run_id = time.strftime("%Y-%m-%d %H:%M:%S")
        logger = logging.getLogger('simulator_run_log')
        logger.setLevel(logging.DEBUG)

        debug_file = logging.FileHandler('logs/{run_id}.debug'.format(run_id = run_id))
        debug_file.setLevel(logging.DEBUG)
        info_file = logging.FileHandler('logs/{run_id}.info'.format(run_id = run_id))
        info_file.setLevel(logging.INFO)

        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        debug_file.setFormatter(formatter)
        info_file.setFormatter(formatter)

        logger.addHandler(debug_file)
        logger.addHandler(info_file)

        return logger

    @staticmethod
    def getDefaultOptions():

        options = Bunch()

        options.World = Bunch()
        options.World.obstaclesInnerFraction = 0.99
        options.World.randomSeed = 40
        options.World.percentObsDensity = 20
        options.World.nonRandomWorld = True
        options.World.circleRadius = 1.0
        options.World.scale = 10.0

        options.Sensor = Bunch()
        options.Sensor.rayLength = 20
        options.Sensor.numRays = 20

        options.controller = Bunch()
        options.controller.type = CubicObjectiveController
        options.controller.u_max = 4

        options.Car = Bunch()
        options.Car.velocity = 16

        options.dt = 0.05

        options.runTime = Bunch()
        options.runTime.defaultControllerTime = 100

        return options

    def initializeColorMap(self):
        self.colorMap = dict()
        self.colorMap['default'] = [0,1,0]

    def initialize(self):

        self.controllerTypeOrder = ['default']

        self.Sensor = SensorObj(num_rays=self.options.Sensor.numRays,
                                ray_length=self.options.Sensor.rayLength)

        self.Controller = self.options.controller.type(self.Sensor, u_max=self.options.controller.u_max)

        self.Car = CarPlant(controller=self.Controller,
                            velocity=self.options.Car.velocity)

        # create the things needed for simulation
        om.removeFromObjectModel(om.findObjectByName('world'))
        self.world = World.buildCircleWorld(percentObsDensity=self.options.World.percentObsDensity,
                                            circleRadius=self.options.World.circleRadius,
                                            nonRandom=self.options.World.nonRandomWorld,
                                            scale=self.options.World.scale,
                                            randomSeed=self.options.World.randomSeed,
                                            obstaclesInnerFraction=self.options.World.obstaclesInnerFraction)

        om.removeFromObjectModel(om.findObjectByName('robot'))
        self.robot, self.frame = World.buildPlane()
        self.locator = World.buildCellLocator(self.world.visObj.polyData)
        self.Sensor.setLocator(self.locator)
        self.frame = self.robot.getChildFrame()
        self.frame.setProperty('Scale', 3)
        self.frame.setProperty('Edit', True)
        self.frame.widget.HandleRotationEnabledOff()
        rep = self.frame.widget.GetRepresentation()
        rep.SetTranslateAxisEnabled(2, False)
        rep.SetRotateAxisEnabled(0, False)
        rep.SetRotateAxisEnabled(1, False)

        self.defaultControllerTime = self.options.runTime.defaultControllerTime

        print "Finished initialization"


    def runSingleSimulation(self, controllerType='default', simulationCutoff=None):


        self.setRandomCollisionFreeInitialState()
        self.logger.info('Robot started from state {s}'.format(s = self.Car.state))


        currentCarState = np.copy(self.Car.state)
        self.setRobotFrameState(currentCarState)
        currentRaycast = self.Sensor.raycast_all(self.frame)

        # record the reward data
        runData = dict()
        startIdx = self.counter


        while (self.counter < self.numTimesteps - 1):
            idx = self.counter
            self.stateOverTime[idx,:] = currentCarState
            self.setRobotFrameState(currentCarState)
            currentRaycast = self.Sensor.raycast_all(self.frame)
            self.raycastData[idx,:] = currentRaycast


            controlInput = self.Controller.compute_u(raycastDistance=currentRaycast)
            self.logger.debug('Control input was {u}'.format(u = controlInput))
            self.controlInputData[idx] = controlInput

            nextCarState = self.Car.simulate_one_step(control_input=controlInput, dt=self.options.dt)

            # want to compute nextRaycast so we can do the SARSA algorithm
            self.setRobotFrameState(nextCarState)
            nextRaycast = self.Sensor.raycast_all(self.frame)

            #bookkeeping
            currentCarState = nextCarState
            currentRaycast = nextRaycast
            self.counter+=1

            # break if we are in collision
            if self.checkInCollision(nextRaycast):
                self.logger.info('Robot collided at timestep {t}'.format(t = self.counter))
                break

            if self.counter >= simulationCutoff:
                break


        # fill in the last state by hand
        self.stateOverTime[self.counter,:] = currentCarState
        self.raycastData[self.counter,:] = currentRaycast


        # this just makes sure we don't get stuck in an infinite loop.
        if startIdx == self.counter:
            self.counter += 1

        return runData

    def runBatchSimulation(self, endTime=None, dt=0.05):

        # for use in playback
        self.dt = self.options.dt

        self.endTime = self.defaultControllerTime # used to be the sum of the other times as well

        self.t = np.arange(0.0, self.endTime, dt)
        maxNumTimesteps = np.size(self.t)
        self.stateOverTime = np.zeros((maxNumTimesteps+1, 3))
        self.raycastData = np.zeros((maxNumTimesteps+1, self.Sensor.numRays))
        self.controlInputData = np.zeros(maxNumTimesteps+1)
        self.numTimesteps = maxNumTimesteps

        self.controllerTypeOrder = ['default']
        self.counter = 0
        self.simulationData = []
    
        self.initializeStatusBar()

        self.idxDict = dict()
        numRunsCounter = 0


        self.idxDict['default'] = self.counter
        loopStartIdx = self.counter
        simCutoff = min(loopStartIdx + self.defaultControllerTime/dt, self.numTimesteps)
        
        while ((self.counter - loopStartIdx < self.defaultControllerTime/dt) and self.counter < self.numTimesteps-1):
            self.printStatusBar()
            startIdx = self.counter
            runData = self.runSingleSimulation(controllerType='default',
                                               simulationCutoff=simCutoff)
            runData['startIdx'] = startIdx
            runData['controllerType'] = "default"
            runData['duration'] = self.counter - runData['startIdx']
            runData['endIdx'] = self.counter
            runData['runNumber'] = numRunsCounter
            numRunsCounter+=1
            self.simulationData.append(runData)

        # BOOKKEEPING
        # truncate stateOverTime, raycastData, controlInputs to be the correct size
        self.numTimesteps = self.counter + 1
        self.stateOverTime = self.stateOverTime[0:self.counter+1, :]
        self.raycastData = self.raycastData[0:self.counter+1, :]
        self.controlInputData = self.controlInputData[0:self.counter+1]
        self.endTime = 1.0*self.counter/self.numTimesteps*self.endTime



    def initializeStatusBar(self):
        self.numTicks = 10
        self.nextTickComplete = 1.0 / float(self.numTicks)
        self.nextTickIdx = 1
        print "Simulation percentage complete: (", self.numTicks, " # is complete)"
    
    def printStatusBar(self):
        fractionDone = float(self.counter) / float(self.numTimesteps)
        if fractionDone > self.nextTickComplete:

            self.nextTickIdx += 1
            self.nextTickComplete += 1.0 / self.numTicks

            timeSoFar = time.time() - self.startSimTime 
            estimatedTimeLeft_sec = (1 - fractionDone) * timeSoFar / fractionDone
            estimatedTimeLeft_minutes = estimatedTimeLeft_sec / 60.0

            print "#" * self.nextTickIdx, "-" * (self.numTicks - self.nextTickIdx), "estimated", estimatedTimeLeft_minutes, "minutes left"


    def setRandomCollisionFreeInitialState(self):
        tol = 5

        while True:
            
            x = np.random.uniform(self.world.Xmin+tol, self.world.Xmax-tol, 1)[0]
            y = np.random.uniform(self.world.Ymin+tol, self.world.Ymax-tol, 1)[0]
            theta = np.random.uniform(0,2*np.pi,1)[0]
            
            state = np.array([x, y, theta])

            self.Car.set_car_state(state)
            self.setRobotFrameState(state)

            if not self.checkInCollision():
                break

        return state

    def setupPlayback(self):

        self.timer = TimerCallback(targetFps=30)
        self.timer.callback = self.tick

        playButtonFps = 1.0/self.options.dt
        print "playButtonFPS", playButtonFps
        self.playTimer = TimerCallback(targetFps=playButtonFps)
        self.playTimer.callback = self.playTimerCallback
        self.sliderMovedByPlayTimer = False

        panel = QtGui.QWidget()
        l = QtGui.QHBoxLayout(panel)

        playButton = QtGui.QPushButton('Play/Pause')
        playButton.connect('clicked()', self.onPlayButton)

        slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        slider.connect('valueChanged(int)', self.onSliderChanged)
        self.sliderMax = self.numTimesteps
        slider.setMaximum(self.sliderMax)
        self.slider = slider

        l.addWidget(playButton)
        l.addWidget(slider)

        w = QtGui.QWidget()
        l = QtGui.QVBoxLayout(w)
        l.addWidget(self.view)
        l.addWidget(panel)
        w.showMaximized()

        self.frame.connectFrameModified(self.updateDrawIntersection)
        self.updateDrawIntersection(self.frame)

        applogic.resetCamera(viewDirection=[0.2,0,-1])
        self.view.showMaximized()
        self.view.raise_()
        panel = screengrabberpanel.ScreenGrabberPanel(self.view)
        panel.widget.show()

        cameracontrolpanel.CameraControlPanel(self.view).widget.show()

        elapsed = time.time() - self.startSimTime
        simRate = self.counter/elapsed
        print "Total run time", elapsed
        print "Ticks (Hz)", simRate
        print "Number of steps taken", self.counter
        self.app.start()

    def run(self, launchApp=True):
        self.counter = 1
        self.runBatchSimulation()

        if launchApp:
            self.setupPlayback()

    def updateDrawIntersection(self, frame):

        origin = np.array(frame.transform.GetPosition())
        #print "origin is now at", origin
        d = DebugData()

        sliderIdx = self.slider.value

        controllerType = self.getControllerTypeFromCounter(sliderIdx)
        colorMaxRange = self.colorMap[controllerType]

        for i in xrange(self.Sensor.numRays):
            ray = self.Sensor.rays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            #print "rayTransformed is", rayTransformed
            intersection = self.Sensor.raycast(origin, origin + rayTransformed * self.Sensor.rayLength)

            if intersection is not None:
                d.addLine(origin, intersection, color=[1,0,0])
            else:
                d.addLine(origin, origin+rayTransformed*self.Sensor.rayLength, color=colorMaxRange)

        vis.updatePolyData(d.getPolyData(), 'rays', colorByName='RGB255')

    def getControllerTypeFromCounter(self, counter):
        name = self.controllerTypeOrder[0]

        for controllerType in self.controllerTypeOrder[1:]:
            if counter >= self.idxDict[controllerType]:
                name = controllerType

        return name

    def setRobotFrameState(self, q):
        (x, y, theta) = q
        t = vtk.vtkTransform()
        t.Translate(x,y,0.0)
        t.RotateZ(np.degrees(theta))
        self.robot.getChildFrame().copyFrame(t)

    def checkInCollision(self, raycast_distances=None):
        """

        :param raycast_distances:
        :return: True if we are in collision, and false otherwise.
        """
        if raycast_distances is None:
            self.setRobotFrameState(self.Car.state)
            raycast_distances = self.Sensor.raycast_all(self.frame)

        return np.min(raycast_distances) < self.collisionThreshold

    # TODO: Ask Pete what the purpose of tick is
    def tick(self):
        x = np.sin(time.time())
        y = np.cos(time.time())
        state = np.array([x, y, 0.0])

        self.setRobotFrameState(state)
        if (time.time() - self.playTime) > self.endTime:
            self.playTimer.stop()

    # just increment the slider, stop the timer if we get to the end
    def playTimerCallback(self):
        self.sliderMovedByPlayTimer = True
        currentIdx = self.slider.value
        nextIdx = currentIdx + 1
        self.slider.setSliderPosition(nextIdx)
        if currentIdx >= self.sliderMax:
            print "reached end of tape, stopping playTime"
            self.playTimer.stop()

    def onSliderChanged(self, value):
        if not self.sliderMovedByPlayTimer:
            self.playTimer.stop()
        numSteps = len(self.stateOverTime)
        idx = int(np.floor(numSteps*(1.0*value/self.sliderMax)))
        idx = min(idx, numSteps-1)
        self.setRobotFrameState(self.stateOverTime[idx])
        self.sliderMovedByPlayTimer = False

    def onPlayButton(self):

        if self.playTimer.isActive():
            self.onPauseButton()
            return

        print 'play'
        self.playTimer.start()
        self.playTime = time.time()

    def onPauseButton(self):
        print 'pause'
        self.playTimer.stop()

    # TODO: Ask Pete how all these save to file things work.
    def saveToFile(self, filename):

        # should also save the run data if it is available, i.e. stateOverTime, rewardOverTime

        filename = 'data/' + filename + ".out"
        my_shelf = shelve.open(filename,'n')

        my_shelf['options'] = self.options

        my_shelf['simulationData'] = self.simulationData
        my_shelf['stateOverTime'] = self.stateOverTime
        my_shelf['raycastData'] = self.raycastData
        my_shelf['controlInputData'] = self.controlInputData
        my_shelf['numTimesteps'] = self.numTimesteps
        my_shelf['idxDict'] = self.idxDict
        my_shelf['counter'] = self.counter
        my_shelf.close()


    @staticmethod
    def loadFromFile(filename):
        filename = 'data/' + filename + ".out"
        sim = Simulator(verbose=False)

        my_shelf = shelve.open(filename)
        sim.options = my_shelf['options']
        sim.initialize()

        sim.simulationData = my_shelf['simulationData']
        sim.stateOverTime = np.array(my_shelf['stateOverTime'])
        sim.raycastData = np.array( my_shelf['raycastData'])
        sim.controlInputData = np.array(my_shelf['controlInputData'])
        sim.numTimesteps = my_shelf['numTimesteps']
        sim.idxDict = my_shelf['idxDict']
        sim.counter = my_shelf['counter']

        my_shelf.close()

        return sim




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='interpret simulation parameters')
    parser.add_argument('--percentObsDensity', type=float, nargs=1, default=[10])
    parser.add_argument('--endTime', type=int, nargs=1, default=[40])
    parser.add_argument('--nonRandomWorld', action='store_true', default=False)
    parser.add_argument('--circleRadius', type=float, nargs=1, default=0.7)
    parser.add_argument('--worldScale', type=float, nargs=1, default=1.0)
    
    argNamespace = parser.parse_args()
    percentObsDensity = argNamespace.percentObsDensity[0]
    endTime = argNamespace.endTime[0]

    nonRandomWorld = argNamespace.nonRandomWorld
    circleRadius = argNamespace.circleRadius[0]
    worldScale = argNamespace.worldScale[0]
    
    sim = Simulator(endTime=endTime)
    sim.run()


