#!/usr/bin/env python
__author__="athanasia sapountzi"

import roslib, warnings, rospy, math, pickle, scipy.stats
from gridfit import gridfit
import numpy as np
import scipy.io as sio
import scipy.special
import matplotlib.pyplot as plt
import mytools as mt #DBSCAN function and perquisites are stored here

from myhog import hog
from sensor_msgs.msg import LaserScan
from scipy.stats.mstats import zscore

ccnames =['gray', 'black', 'violet', 'blue', 'cyan', 'rosy', 'orange', 'red', 'green', 'brown', 'yellow', 'gold']
cc  =  ['#808080',  'k',  '#990099', '#0000FF', 'c','#FF9999','#FF6600','r','g','#8B4513','y','#FFD700']
wall_flag=0
fr_index=1
z=0
z_scale= float(5*40) / float(3600)
w_index=1
counter=0
limit=40

plt.ion()
def laser_listener():

    global timewindow,range_limit

    rospy.init_node('laser_listener', anonymous=True)
    sub_topic = "scan"
    print 'insert timewindow'
    timewindow=input()
    print 'set max distance'
    range_limit=input()

    print('Subscribing to: %s' % sub_topic)
    rospy.Subscriber(sub_topic,LaserScan,online_test)
    rospy.spin()

def online_test(laser_data):

    global wall_flag , wall , fr_index ,  intens ,w_index,phi,sampling
    global phi, mybuffer, z, zscale, gaussian,timewindow , wall_cart,ax,fig1, kat

    if wall_flag==0:
        if w_index==1:
            #print 'Reduce points by 2? 1/0'
           # if input()==1 :
            sampling=np.arange(0,len(np.array(laser_data.ranges)),2)#apply sampling e.g every 2 steps
            #else :
            #    sampling=np.arange(0,len(np.array(laser_data.ranges)),1)
            R,intens,F,gaussian=loadfiles()
            wall=np.array(laser_data.ranges)
            mybuffer=wall
            filter=np.where(wall>=range_limit)
            wall[filter]=range_limit
            w_index=w_index+1
        if w_index<limit:
            wall=np.array(laser_data.ranges)
            filter=np.where(wall>=range_limit)
            wall[filter]=range_limit
            mybuffer=np.vstack((mybuffer,wall ))  #  add to buffer with size=(wall_index x 360)
            w_index=w_index+1
        if w_index==limit:
            mybuffer=np.vstack((mybuffer,wall ))
            phi=np.arange(laser_data.angle_min,laser_data.angle_max,laser_data.angle_increment)[sampling]
            wall=(np.min(mybuffer, axis=0)[sampling])-0.1 #select min of measurements
            wall_cart=np.array(pol2cart(wall,phi,0) ) #convert to Cartesian
            wall_flag=1
            kat,ax=initialize_plots(wall_cart)
            print 'walls set...'
        
    else:
        ranges=np.array(laser_data.ranges)[sampling]
        filter = np.where(ranges < wall) # filter out walls
        ranges = ranges[filter]
        theta = phi[filter]


        if (len(ranges)>3): #each scan should consist of at least 3 points to be valid

            C=np.array(pol2cart(ranges,theta,z) ) #convert to Cartesian

            if (fr_index ==1 ):
                mybuffer=C
            else :
                mybuffer=np.concatenate((mybuffer,C), axis=0 )  #  add to

            if (fr_index == timewindow ):

                mybuffer=mybuffer[np.where( mybuffer[:,0] > 0.2),:][0] #mishits safety margin
                mybuffer=mybuffer[np.where( mybuffer[:,0] < 5),:][0]#ignore distant points

                if len(mybuffer>3): #at least 3 points are needed to form a cluster
                    clustering(mybuffer)
                fr_index=0
                z=- z_scale
            z = z + z_scale
            fr_index=fr_index+1

def pol2cart(r,theta,zed):

    #metatropi kylindrikon syntentagmenon se kartesianes
    x=np.multiply(r,np.cos(theta))
    y=np.multiply(r,np.sin(theta))
    z=np.ones(r.size)*zed
    C=np.array([x,y,z]).T
    return C

def loadfiles():
    
    #Load intensity-range-angle data and classifier
    mat=sio.loadmat('ideal_data.mat')
    ranges=mat.get('ranges')
    intensities=mat.get('intensities')
    angle_min=mat.get('angle_min')
    angle_max=mat.get('angle_max')
    angle_increment=mat.get('angle_increment')
    theta=np.arange(angle_min,angle_max,angle_increment)

    classifier = pickle.load( open( "Gaussian_NB_classifier.p", "rb" ) )

    return ranges,intensities,theta,classifier

def initialize_plots(wall_cart):
    global fig1

    temp=plt.figure()
    plot2d = temp.add_subplot(111)
    plot2d.set_xlabel('Vertical distance')
    plot2d.set_ylabel('Robot is here')
    plot2d.plot(wall_cart[:,0],wall_cart[:,1])

    fig1=plt.figure()
    plot3d= fig1.gca(projection='3d')
    plot3d.set_xlabel('X - Distance')
    plot3d.set_ylabel('Y - Robot')
    plot3d.set_zlabel('Z - time')

    plt.show()
    return plot2d,plot3d

def clustering(clear_data):

    global cc, ccnames, fig1
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    hogs=[]
    colors=[]
    vcl=[] #Valid Cluster Labels 
    valid_flag=0 #this flag is only set if we have at leat one valid cluster
    Eps, cluster_labels= mt.dbscan(clear_data,3) # DB SCAN
    #print  len(clear_data),' points in ', np.amax(cluster_labels),'clusters'
    #print 'Eps = ', Eps, ', outliers=' ,len(np.where(cluster_labels==-1))
    max_label=int(np.amax(cluster_labels))

    [xi,yi,zi] = [clear_data[:,0] , clear_data[:,1] , clear_data[:,2]]
    fig1.clear()
    
    #scatter_all(xi,yi,zi,cluster_labels)#optional 3d scatter plot of all clusters

    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>40 :
            #ax.scatter(xi[filter],yi[filter], zi[filter], 'z', 30, cc[k-1]) #this can be commented out
            valid_flag=1
            #print 'extracting surface for ',ccnames[k-1],' cluster '
            vcl.append(k)
            colors.append(ccnames[k-1])
            grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            grid=grid-np.amin(grid)
            hogs.append(hog(grid))  #extract hog features

    fig1.show()

    update_plots(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl)
    
def scatter_all(xi,yi,zi,cluster_labels):
    
    global cc
    fig2 = plt.figure()
    ax2 = fig2.gca(projection='3d')
    ax2.set_xlabel('X - Distance')
    ax2.set_ylabel('Y - Robot')
    ax2.set_zlabel('Z - time')

    max_label=int(np.amax(cluster_labels))

    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>40 :
            ax2.scatter(xi[filter],yi[filter], zi[filter], 'z', 30, cc[k-1])

    fig2.pause(0.00001)

def update_plots(flag,hogs,xi,yi,zi,cluster_labels,vcl):
    
    global kat,fig1,ax,wall_cart,gaussian,counter
    temp=[]
    if flag==1:
        kat.clear()
        kat.plot(wall_cart[:,0],wall_cart[:,1])
        if np.array(hogs).shape==(1,36):
            temp=zscore(np.array(hogs)[0])
        else:
            for i in range(0,len(hogs)):
                temp.append(zscore(np.array(hogs[i])))

        results= gaussian.predict(np.array(temp)) #CLASSIFICATION

        cnt=0
        for k in vcl:

            filter=np.where(cluster_labels==k)
            
            [x,y,zed] = [xi[filter] , yi[filter] , zi[filter]]

            if results[cnt]==1:
                kat.scatter(x,y,s=20, c='r')
                #ax.scatter(x,y, zed, 'z', 30, c='r') #human
                #fig1.add_axes(ax)
            else:
                kat.scatter(x,y,s=20, c='b')
                #ax.scatter(x,y, zed, 'z', 30, c='b') #object
                #fig1.add_axes(ax)
            cnt=cnt+1
        plt.pause(0.0001)


        counter=counter+cnt



if __name__ == '__main__':
    laser_listener()
