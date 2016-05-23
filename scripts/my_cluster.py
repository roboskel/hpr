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
	self._computeMedian()


    #add a new point to this cluster
    def addPoint(self, new_point):
	np.append(self.points, new_point)
	self._computeMedian()
	self.numPts = self.numPts + 1

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

    #add to the current cluster new points and update the median core
    #outlier: <DBscanCluster> instance. Represents an 'outlier cluster'
    def concat(self, outlier):
	self.points = np.concatenate((self.points, outlier.getPoints()), axis=0)
	self.numPts = self.numPts + outlier.getNumPts()
	self._computeMedian()


