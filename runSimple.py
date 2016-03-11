__author__ = 'manuelli'
from simulator import Simulator

sim = Simulator()

sim.options.World.randomSeed = 10
sim.options.World.percentObsDensity = 40

sim.initialize()
sim.run()