import director.vtkAll as vtk

import numpy as np
import director.objectmodel as om
import director.visualization as vis
from director.debugVis import DebugData


class MappingObj(object):

    def __init__(self):
        self.numX = 5
        self.numY = 5
        self.numZ = 5

        self.voxelSize = 1.0 # in meters

        self.occMap = np.zeros((self.numX, self.numY, self.numZ))
        self.debugFill()

    def debugFill(self):

        d = DebugData()

        print "got here"

        for i in xrange(self.numX):
            print "and here"
            for j in xrange(self.numY):
                print "and even here"
                for k in xrange(self.numZ):
                    firstEndpt = (i,j,k-0.1)
                    secondEndpt = (i,j,k+0.1)
                    d.addLine(firstEndpt, secondEndpt, radius=0.2)
                    obj = vis.showPolyData(d.getPolyData(), 'world')


    def checkOccupied(self, x, y, z):

        if self.occMap[x,y,z] != 1:
            self.occMap[x,y,z] = 1
            self.drawOccupied(x, y, z)

    def drawOccupied(self, x, y, z):

        d = DebugData()

        firstEndpt = (x,y,z-0.1)
        secondEndpt = (x,y,z+0.1)

        #d.addLine(firstEndpt, secondEndpt, radius=0.2)
        #obj = vis.showPolyData(d.getPolyData(), 'world')
        
        