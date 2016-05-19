import math
import numpy as np
import heapq

current_clusters = []

def onlineDBscan(x):

    [m,n] = x.shape()

    for i in range(0, m-1):




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


    Eps=epsilon(x,k)

    [m,n]=x.shape
    a=np.array(range(len(x)))

    #insert at the first position of x-array the index of each point, which represents its unique id
    x=np.insert(x,0,a,1)
    [m,n]=x.shape
    point_type=np.ones(m)
    cluster_label=np.ones(m)
    no=1
    touched=np.zeros(m)
     

    for i in range(0,m-1):

        if touched[i]==0:

            ob=x[i,:]
            D=dist(ob[1:n] ,x[:,1:n])
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
                    D=dist( ob[1:n],x[:,1:n] )
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

    #heapq.heappush(heap_labels, cluster_label)

    return cluster_label

	






