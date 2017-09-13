
import scipy.spatial.distance as dist

#Class that is used to keep necessary information for every traced_cluster about. It is useful for the walk_analysis procedure.

#Attributes:
#    - new: defines whether the human is appeared for the first time
#    - hum_id: an identifier of the human
#    - tot_dist: represents the distance that the human walked until now
#    - tot_time: represents the time that the human walked fot <tot_dist> meters
#    - medX: list of median data in x-dimension
#    - medY: list of median data in y-dimension
#    - prevMedian: the previous median (x,y)-point
#    - timestamp: the time that the human started to walk, as it is declared from ROS.time
#    - stable: checks whether the human is motionless or not
#    - stopCounter: counter for how many frames the human is motionless
#
#Info:
#   - Use of euclidean distance for the distance calculation between the points

class WalkTrack:

    threshold = 0.05
    num_frames = 7

    def __init__(self, hum_id):
        self.new = True
        self.hum_id = hum_id
        self.tot_dist = 0.0
        self.tot_time = 0.0
        self.medX = []
        self.medY = []
        self.prevMedian = []
        self.timestamp = 0.0
        self.stable = False
        self.stopCounter = 0

    def is_new(self):
        return self.new

    def get_id(self):
        return self.hum_id

    def get_distance(self):
        return self.tot_dist

    def set_distance(self, distance):
        self.tot_dist += distance

    def get_time(self):
        return self.tot_time

    def set_time(self,time):
        self.tot_time += time

    def get_timestamp(self):
        return self.timestamp

    def set_timestamp(self, timestamp):
        self.timestamp = timestamp

    def is_stable(self):
        return self.stable

    def set_stable(self, current_condition):
        self.stable = current_condition

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


    def compute_error(self, x, y):
        if (dist.euclidean([x, y], self.prevMedian) <= self.threshold):
            self.stopCounter += 1

        if self.stopCounter == self.num_frames:
            return True
        return False


    def initialise(self):
        self.tot_dist = 0.0
        self.tot_time = 0.0
        self.medX = []
        self.medY = []
        self.prevMedian = []
        self.stable = False
        self.stopCounter = 0

