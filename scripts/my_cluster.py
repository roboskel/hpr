class DBscanCluster:

    def __init__(self, clId, numPts, points):
	self.clId = clId
	self.meanCore = np.zeros(points.shape[1])
	self.numPts = numPts
	self.points = points


    #add a new point to this cluster
    def add_point(self, new_point):
	self.points.append(new_point)
	_compute_median(self)

    #compute the median point of the set of cluster
    def _compute_median(self):
	for i in range(0, self.points.shape[1]):
	    xmed = np.median(points[i])
	self.meanCore = np.array(xmean, ymean, zmean)
