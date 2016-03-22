__author__ = 'manuelli'
from simulator import Simulator

sim = Simulator()

sim.options.World.randomSeed = 10
sim.options.World.percentObsDensity = 40
sim.options.dt = 1.0/90.0
sim.options.runTime.defaultControllerTime = 300

sim.initialize()
sim.run()