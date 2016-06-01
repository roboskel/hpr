import scipy.spatial.distance as dist

#Class that is used to keep necessary information for every traced_cluster-human about the walk_speed procedure.

#Attributes:
#    - new: defines whether the human is appeared for the first time
#    - tot_dist: represents the distance that the human walked until now
#    - tot_time: represents the time that the human walked fot <tot_dist> meters
#    - medX: list of median data in x-dimension
#    - medY: list of median data in y-dimension
#    - prevMedian: the previous median (x,y)-point 
#
#Info:
#   - Use of euclidean distance for the distance calculation between the points

class WalkTrack:

    def __init__(self):
        self.new = True
        self.tot_dist = 0.0
        self.tot_time = 0.0
        self.medX = []
        self.medY = []
        self.prevMedian = []

    def is_new(self):
        return self.new

    def get_distance(self):
        return self.tot_dist

    def set_distance(self, distance):
        self.tot_dist += distance

    def get_time(self):
        return self.tot_time

    def set_time(self,time):
        self.tot_time += time

    def addX(self, x):
        self.medX.append(x)

    def addY(self, y):
        self.medY.append(y)

    def set_prevMedian(self, x, y):
        self.prevMedian = [x,y]
        self.new = False

    def empty(self):
        if len(self.prevMedian) == 0:
            return True
        return False

    def add_distance(self, x, y):
        self.tot_dist = self.tot_dist + dist.euclidean([x, y], self.prevMedian)

    def initialise(self):
        self.tot_dist = 0.0
        self.tot_time = 0.0
        self.medX = []
        self.medY = []
        self.prevMedian = []


