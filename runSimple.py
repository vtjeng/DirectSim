__author__ = 'manuelli'
from simulator import Simulator

sim = Simulator(verbose=False)

sim.options.World.randomSeed = 8
sim.options.World.percentObsDensity = 4

sim.initialize()
sim.run()