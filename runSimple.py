__author__ = 'manuelli'
from simulator import Simulator
from bunch import *

options = Bunch()
options.World = Bunch()
options.World.randomSeed = 10
options.World.percentObsDensity = 40
options.dt = 1.0/90.0
options.runTime = Bunch()
options.runTime.defaultControllerTime = 300

sim = Simulator(options=options)

sim.run()