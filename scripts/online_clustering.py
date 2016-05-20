import math
import numpy as np
import Queue
import scipy.spatial.distance as distance
from scipy import special
import my_cluster as cl

prev_clusters = []	#an array that contains the <DBscanClusters> that are constructed in a previous phase. (Necessary for the online clustering)
cluster_label = []	#the labels of the clusters until now 
cluster_queue =  Queue.Queue(5)		#maximum size of the queue: 5 timewindows
Eps = 0.5 #default


def onlineDBscan(points, minPts):
    global prev_clusters, cluster_label
    global Eps
    temp_outliers = [] 	#a temp list to put noise data that may be part of a cluster in the future

    [m,n] = points.shape
    max_id = int(np.amax(cluster_label))

    for p in range(0, m-1):
	dist_array = np.zeros(len(prev_clusters))

	for i in range(0, len(prev_clusters)):
	    #get the euclidean distance of the point and of the median core of each cluster
	    dist_array[i] = distance.euclidean(prev_clusters[i].getMedian(), points[p])	

	min_dist = np.amin(dist_array)

	border, min_index = existMinDistance(temp_outliers, points[p], min_dist)

	#the minimum distance is from an already formed cluster
	if min_index == -1:
	    min_index = dist_array.index(min_dist)
	    K = prev_clusters[min_index]

	    #condition where the point belongs to this cluster
	    if dist(points[p], K.getPoints()) <= Eps:
	    	K.addPoint(points[p])
	    else:
		temp_outliers.append(cl.DBscanCluster(++max_id, points[p]))

	else:

	    #this point is a border of this cluster but is reachable-connected with a noise point too
	    if border:
		pot_index = dist_array.index(min_dist)
	    	#TODO
		#update the outlier cluster with the point
		#concatenate the two clusters to one -> concatenate() in my_cluster.py
		#remove the outlier cluster from the temp list
	    else:
		potential_cl = temp_outliers[min_index]
		#it forms a new cluster 
		#	-> put the new point and update the respective cluster arrays
		if ( (dist(points[p], potential_cl.getMedian()) <= Eps) & (potential_cl.getMinPts() +1 >= minPts) ):
		    potential_cl.addPoint(points[p])
		    prev_clusters.append(potential_cl)
		    del temp_outliers[min_index]

		#this point belongs to the outlier cluster but a cluster is not formed yet
		elif ( (dist(points[p], potential_cl.getMedian()) <= Eps) & (potential_cl.getMinPts() +1 < minPts) ):
		    potential_cl.addPoint(points[p])

		else:
		    # new outlier
		    if (dist(points[p], potential_cl.getMedian()) > Eps):
			temp_outliers.append(cl.DBscanCluster(++max_id, points[p]))
		
    #TODO: update the cluster_label
    return cluster_label


#It checks if there is a minimum distance in the set of outliers. 
#	-> it can indicate the creation of a new cluster
def existMinDistance(outliers, point, min_dist):
    index = -1
    flag = False

    for i in range(0,len(outliers)):
	temp_d = distance.euclidean(point, outliers[i].getMedian())

	# there is a minimum distance in the outliers
	if  temp_d < min_dist:
	    min_dist = temp_d
	    index = i

	    if flag:
		flag = False

	    #case of border point (concatenation of clusters)
	    if temp_d == min_dist:
	    	flag = True    


    return flag, index


def epsilon(x,k):
# Function: [Eps]=epsilon(x,k)
# Aim:
# Analytical way of estimating neighborhood radius for DBSCAN
# Input:
# x - data matrix (m,n); m-objects, n-variables
# k - number of objects in a neighborhood of an object
# (minimal number of objects considered as a cluster)
    [m,n]=x.shape
    numerator = float( np.prod(x.max(0)-x.min(0) )*k*special.gamma(0.5*n+1) )
    denominator = float( m*math.sqrt(math.pi ** n) )
    power = float(1)/float(n)
    Eps=( numerator / denominator )**power

    return Eps

def dist(i,x) :

# function: [D]=dist(i,x)
# Aim:
# Calculates the Euclidean distances between the i-th object and all objects in x
# Input:
# i - an object (1,n)
# x - data matrix (m,n); m-objects, n-variables
# Output:
# D - Euclidean distance (m,1)
    x=np.array(x)
    [m,n]=x.shape
    D=np.zeros(m)

    if n==1 :
        D = abs( np.array(i) - x ).T
    else:
        D=np.sqrt( np.sum(( (i - x) ** 2 ), axis=1) )

    return D


def dbscan(x,k):

# Function: [class,type]=dbscan(x,k,Eps)
# Aim:
# Clustering the data with Density-Based Scan Algorithm with Noise (DBSCAN)
# Input:
# x - data set (m,n); m-objects, n-variables
# k - number of objects in a neighborhood of an object
# (minimal number of objects considered as a cluster)
# 
# Output:
# Eps - neighborhood radius, if not known avoid this parameter or put []
# type - vector specifying type of the i-th object
# (core: 1, border, outlier: -1)

    global cluster_label
    global Eps

    Eps=epsilon(x,k)

    [m,n]=x.shape
    a=np.array(range(len(x)))

    #insert at the first position of x-array the index of each point, which represents its unique id
    #x=np.insert(x,0,a,1)
    [m,n]=x.shape
    point_type=np.ones(m)
    cluster_label=np.ones(m)
    no=1
    touched=np.zeros(m)
     

    for i in range(0,m-1):

        if touched[i]==0:

            ob=x[i,:]
            D=dist(ob[0:n] ,x[:,0:n])
	    #array of index-points where object i has dist<=eps 
            ind=np.array(np.where(D<=Eps))[0]

            if len(ind)>1 and len(ind)<k+1:

                point_type[i]=0
                cluster_label[i]=0

            if len(ind)==1:

                point_type[i]=-1
                cluster_label[i]=-1
                touched[i]=1

            if len(ind)>=k+1:

                point_type[i]=1
                cluster_label[ind]=np.ones([len(ind),1])*no

                while len(ind)!=0:
                    ob=x[ind[0]]
                    touched[ind[0]]=1
                    ind=np.delete(ind,0)
                    D=dist( ob[0:n],x[:,0:n] )
                    i1=np.array(np.where(D<=Eps))[0]

                    if len(i1)>1:
                        cluster_label[i1]=no
		    #it is a core point
                    if len(i1)>=k+1:
                        point_type[int(ob[0])]=1
                    else:
                        point_type[int(ob[0])]=0

                    for j in range(0,len(i1)):
                        if touched[i1[j]]==0:
                            touched[i1[j]]=1
                            ind=np.hstack((ind,i1[j]))
                            cluster_label[i1[j]]=no
                no=no+1

    myfilter=np.where(point_type==0 )
    point_type[myfilter]=-1
    cluster_label[myfilter]=-1

    build_clusters(x)
    #heapq.heappush(heap_labels, cluster_label)

    return cluster_label



def build_clusters(points):
    global cluster_label, prev_clusters, cluster_queue
    temp_cl = []

    max_label=int(np.amax(cluster_label))

    for k in range(1,max_label+1) :
        filter=np.where(cluster_label==k)

        if len(filter[0])>40 :
	    prev_clusters.append(cl.DBscanCluster(k, points[filter]))
	    temp_cl.append(points[filter])
    
    cluster_queue.put(temp_cl)






