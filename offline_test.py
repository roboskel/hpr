#!/usr/bin/env python
__author__="athanasia sapountzi"

import  warnings, math, pickle, scipy.stats
from gridfit import gridfit
import numpy as np
import scipy.io as sio
import scipy.special
import matplotlib.pyplot as plt
import mytools as mt #DBSCAN function and perquisites are stored here

from myhog import hog
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

def offline_test():

    global wall_flag , wall , fr_index ,  intens ,w_index,phi,sampling
    global phi, z, zscale, gaussian,timewindow , wall_cart,ax,fig1, kat

    print 'set timewindow in frames:'
    timewindow=input()
    print 'set max frames for wall settting'
    wall_end=input()
    print 'set maximum range'
    range_limit=input()
    filename=input('Enter data file name: ')
    mat=sio.loadmat(filename)
    #mat=sio.loadmat('bagfile_data.mat')
    all_data=mat.get('ranges')
    angle_min=mat.get('angle_min')
    angle_max=mat.get('angle_max')
    angle_increment=mat.get('angle_increment')
    mybuffer=all_data[0]
    
    print 'Reduce points by 2? 1/0'
    if input()==1 :
        sampling=np.arange(0,len(mybuffer),2)#apply sampling e.g every 2 steps
    else :
        sampling=np.arange(0,len(mybuffer),1)

    max_index=len(all_data)
    em_index=0
    print 'loading classifier'
    gaussian=pickle.load( open( "Gaussian_NB_classifier.p", "rb" ) )
    print 'ok'
    
    phi=np.arange(angle_min,angle_max,angle_increment)[sampling]

    for wall_index in range(1,wall_end):

        wall=all_data[fr_index]
        filter=np.where(wall>=range_limit)
        wall[filter]=range_limit

        if (wall_index<wall_end):
            mybuffer=np.vstack((mybuffer,wall ))  #  add to buffer with size=(wall_index x 720)

    mybuffer=mybuffer[:,sampling]

    wall=np.min(mybuffer, axis=0)-0.1 #select min of measurements
    print wall_index
    wall_cart=np.array(pol2cart(wall,phi,0) )[:,0:2] #convert to Cartesian
    kat,ax=initialize_plots(wall_cart)

    print 'walls set...'

    for outer_index in range(wall_index,max_index):
        ranges=all_data[outer_index]#[ind]
        ranges=ranges[sampling]
        filter=np.where(ranges <= wall) # remove walls
        ranges = ranges[filter]
        theta =phi[filter]

        if (len(ranges)<3 ):
            print 'empty scan'
            em_index=em_index+1

        if (len(ranges)>=3): #each scan should consist of at least 3 points to be valid

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
    valid_flag=0 #this flag is only set if we have at least one valid cluster
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
            ax.scatter(xi[filter],yi[filter], zi[filter], 'z', 30, cc[k-1]) #this can be commented out
            valid_flag=1
            #print 'extracting surface for ',ccnames[k-1],' cluster'
            vcl.append(k)
            colors.append(ccnames[k-1])
            grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            grid=grid-np.amin(grid)
            hogs.append(hog(grid))  #extract hog features
 
    update_plots(valid_flag,hogs,xi,yi,zi,cluster_labels,vcl)
    fig1.show()

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
                ax.scatter(x,y, zed, 'z', 30, c='r') #human
                fig1.add_axes(ax)
            else:
                kat.scatter(x,y,s=20, c='b')
                ax.scatter(x,y, zed, 'z', 30, c='b') #object
                fig1.add_axes(ax)
            cnt=cnt+1
        plt.pause(0.0001)

        counter=counter+cnt


if __name__ == '__main__':
    offline_test()
