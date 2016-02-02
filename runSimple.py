__author__ = 'manuelli'
from CarSimulator import Simulator

sim = Simulator(autoInitialize=False, verbose=False)

sim.Sarsa_numInnerBins = 4
sim.Sarsa_numOuterBins = 4
sim.Sensor_rayLength = 10


sim.randomSeed = 8
sim.randomizeControl       = True
sim.percentObsDensity      = 4
sim.nonRandomWorld         = True
sim.circleRadius           = 2.5
sim.worldScale             = 1
sim.defaultControllerTime  = 300

sim.initialize()
sim.run()