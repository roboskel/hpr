import numpy as np

class DBscanCluster:
#This class represents a cluster structure, that will be utilised from the online dbscan algorithm.
#DBscanCluster has the following properties:
#
#Attributes:
#	clId: the id of the cluster (suitable for labeling)
#	points: the points that concist this cluster
#	medCore: a core point that is the median point in the cluster
#	numPts: the number of points in the cluster

    #constructor
    def __init__(self, clId, points):
	self.clId = clId
	self.medCore = np.zeros(points.shape[1])
	self.numPts = points.shape[0]
	self.points = points
        self.sizes = [self.numPts]
	#self._computeMedian()


    #add a new point to this cluster
    def addPoint(self, new_point):
	self.points = np.append(self.points, [new_point], axis = 0)
	#self._computeMedian()
	self.numPts = self.numPts + 1

    #remove some points of the set
    def removePoints(self, point_set):
        self.points = np.delete(self.points, np.s_[0:len(point_set)], 0)

        self.numPts = self.numPts - len(point_set)
        del self.sizes[0]

        if len(self.sizes) == 0:
            self.sizes.append(0)



    #compute the median point of the points' set
    def _computeMedian(self):
	for i in range(0, self.points.shape[1]):
	    self.medCore[i] = np.median(self.points[:,i])


    def getMedian(self):
	return self.medCore

    def getId(self):
	return self.clId

    def getNumPts(self):
	return self.numPts

    def getPoints(self):
	return self.points

    def getSizes(self):
        return self.sizes

    def addSize(self, sz):

        self.sizes.append(sz)
        if self.sizes[0] == 1:
            del self.sizes[0]


    #add to the current cluster new points and update the median core
    #outlier: <DBscanCluster> instance. Represents an 'outlier cluster'
    def concat(self, outlier):
	self.points = np.concatenate((self.points, outlier.getPoints()), axis=0)
        
	self.numPts = self.numPts + outlier.getNumPts()
	#self._computeMedian()


