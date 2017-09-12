import numpy as np

"""
Represents a single track
"""
class HprTrack:
    cl_threshold = 4
    
    def __init__(self, track_id, points, tw_counter):
        self.track_id = track_id
        self.tw_counter = tw_counter
        self.clusters = { tw_counter:points }


    def update(self, points, tw):
        self.clusters.update({ tw:points })
        self.tw_counter = tw
        # remove the clusters that appeared <cl_threshold> timewindows ago
        self.remove_part()

    def remove_part(self):
        key = self.tw_counter - self.cl_threshold
        self.clusters.pop(key, None)

    #Combines the points (x,y,z) of all the available timewindows in single arrays.
    #z dimension changes: 
        #Each timewindow resets the time, thus the time (z dimension) begins from 0.0.
        #In order to combine the points of the available timewindows, the time should increment periodically in each set of timewindow points.
    def get_points(self, z_scale):
        xValues = None  #np array of all values of x-dimension
        yValues = None  #np array of all values of y-dimension
        zValues = None  #np array of all values of z-dimension
        sizeArray = []  #the number of points in each timewindow
        prev_tw = 0     #previous timewindow counter

        cl = self.clusters.items()
        sorted_clusters = sorted(cl)

        for tw_item in sorted_clusters:
            if not (zValues == None):
                z_last = zValues[-1]
                increment = 1

                if not (prev_tw == 0):
                    #the timewindows are not in sequence, thus the time should increment properly
                    if (tw_item[0] - prev_tw) > 1:
                        increment = prev_tw - tw_item[0]
                
                xValues = np.append(xValues, tw_item[1][0])
                yValues = np.append(yValues, tw_item[1][1])

                zArray = [z+z_last+(increment*z_scale) for z in tw_item[1][2]]
                zValues = np.append(zValues, zArray)
            else:
                xValues = np.array(tw_item[1][0])
                yValues = np.array(tw_item[1][1])
                zValues = np.array(tw_item[1][2])

            sizeArray.append(len(tw_item[1][0]))
            prev_tw = tw_item[0]

        #print 'HprTack {} :: xValues = {} \n yValues = {} \n zValues = {} \n sizeArray = {}'.format(self.track_id, xValues, yValues, zValues, sizeArray)

        return xValues, yValues, zValues, sizeArray

"""
Holds and handles all the tracks
"""
class Tracker:
    tw_threshold = 5    #a threshold that defines whether a track should be kept or not

    def __init__(self):
        self.track_counter = 0
        self.tw_counter = 0
        self.track_array = []


    def update_track(self, trackId, points):
        track = next( (t for t in self.track_array if t.track_id == trackId), None)

        if not (track == None):
            #print 'Tracker :: ready to update a the track {} and the new tw = {} '.format(track.track_id, self.tw_counter)
            track.update(points, self.tw_counter)


    def add_track(self, points):
        #print 'Tracker :: ready to add a new track to tw = {} with id = {}'.format(self.tw_counter, self.track_counter)
        new_track = HprTrack(self.track_counter, points, self.tw_counter)
        self.track_array.append(new_track)
        self.track_counter += 1


    def update(self):
        self.tw_counter += 1

        #Removes the tracks that are <X> tw away
        for tr in self.track_array:
            if (self.tw_counter - tr.tw_counter) > self.tw_threshold:
                self.track_array.remove(tr)


    def has_previous(self):
        previousTW = self.tw_counter - 1
        return any(t for t in self.track_array if t.tw_counter == previousTW)


    def previous(self):
        previousTW = self.tw_counter - 1
        previousList = [t for t in self.track_array if t.tw_counter == previousTW]

        #print 'Tracker :: previous TW = {}'.format(previousTW)

        if not (len(previousList) == 0):
            previousIds = [t.track_id for t in previousList]
            lastPart = [t.clusters.get(previousTW) for t in previousList]

            return previousIds, lastPart
        return None, None


    def combine_tracks(self, z_scale):
        clusterSeparation = []
        clusterSizes = []
        X_ = None
        Y_ = None
        Z_ = None

        lastPart = [t for t in self.track_array if t.tw_counter == self.tw_counter]

        for i,track in enumerate(self.track_array):
            xValues, yValues, zValues, sizeArray = track.get_points(z_scale)
            clusterSizes = clusterSizes + sizeArray

            if X_ == None:
                X_ = np.array(xValues)
                Y_ = np.array(yValues)
                Z_ = np.array(zValues)
            else:
                X_ = np.append(X_, xValues)
                Y_ = np.append(Y_, yValues)
                Z_ = np.append(Z_, zValues)

            clusterSeparation.append(sum(sizeArray))

        return X_, Y_, Z_, clusterSizes, clusterSeparation


        """
        #for every tracked cluster
        for i in range(0, len(self.track_array)):
            if len(tracks[i]) == 0:
                 index_clusters.append(0)
                 continue
            #initialize x,y,z of trackedCluster_i
            xar = np.array(tracks[i][0][0])
            yar = np.array(tracks[i][0][1])
            zar = np.array(tracks[i][0][2])

            index_clusters.append(len(xar))

            #z dimension changes: 
            #Each timewindow resets the time, thus the time (z dimention) begins from 0.0.
            #In order to combine the points of the available timewindows, the time should increment periodically in each set of timewindow points.
            for j in range(1, len(tracks[i])):
                 xar = np.append(xar,tracks[i][j][0])
                 yar = np.append(yar,tracks[i][j][1])
                 B = tracks[i][j][2].copy()
               
                 index_clusters.append(len(tracks[i][j][0]))

                 increment = zar[len(zar) - 1] + z_scale
                 B[::1]  += increment
                 zar =  np.append(zar,B)    

            final_clusters.append([xar,yar,zar])
        """




