__author__ = 'manuelli'
from simulator import Simulator
from bunch import *
from controller import *

'''
Specify
'''

options = Bunch()
options.World = Bunch()
options.World.randomSeed = 10
options.World.percentObsDensity = 5
options.dt = 1.0/90.0
options.runTime = Bunch()
options.runTime.defaultControllerTime = 300
options.controller = Bunch()
options.controller.type = CountDistancesController
options.controller.u_max = 4


sim = Simulator(options=options)

sim.run()
