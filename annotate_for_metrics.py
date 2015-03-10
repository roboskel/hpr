#!/usr/bin/env python
__author__="athanasia sapountzi"
#python annotate.py <time_window> <wall_set_frames> <max_scan_range> <mat_file_to_use>
import  pickle
import os.path
import sys
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import time
from mytools import princomp,dbscan
from myhog import hog
from scipy import special
from scipy.stats.mstats import zscore
from gridfit import gridfit
from matplotlib import cm


#pre-made classifiers
from sklearn.naive_bayes import GaussianNB

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

ccnames =['GRAY', 'BLACK', 'VIOLET', 'BLUE', 'CYAN', 'ROSY', 'ORANGE', 'RED', 'GREEN', 'BROWN', 'YELLOW', 'GOLD']
cc  =  ['#808080',  'k',  '#990099', '#0000FF', 'c','#FF9999','#FF6600','r','g','#8B4513','y','#FFD700']

fr_index=0
dt = 25;#period in ms (dt between scans)
speed = 5;#human walking speed in km/h
z_scale= float(speed*dt) / float(3600)
z=-z_scale
slot_count=0

slot_touched=0

flag=0
plt.ion()
wall_flag=0
wall_index=1
em_index=0
timewindow=40#timewindow in frames (40 is just an initialization)
filename=''
wall_end = 0
range_limit = 0
annotations_checked = 0
TP = 0
TN = 0
FP = 0
FN = 0

def RepresentsInt(s):
    try: 
        int(s)
        return True
    except ValueError:
        return False
        
def RepresentsFloat(s):
    try:
        float(s)
        return True
    except ValueError:
        return False
        
        
def check_args(arg_list):
    global timewindow, range_limit, wall_end, filename
    print (arg_list)
    filename = str(arg_list[1])
    if not os.path.isfile(filename):
        while True :
            try:
                filename=raw_input('Enter data file name: ')
                if os.path.isfile(filename):
                    break
                else:
                    print 'File does not exist! Try again!'
            except SyntaxError:
                print 'Try again'
    print "File : {0}".format(filename)
    
    #time.sleep(100)
            
def annotate_for_metrics():
    global fr_index , ind ,all_hogs, all_surf,fig1,ax,kat,wall_cart,wall,kat,mat_file
    global z, z_scale, slot_count, human_l ,cluster_l,slot_touched,all_scans
    global timewindow ,slot_data , phi,wall_index,annotations,em_index, filename, wall_end, range_limit
    global classifier_annotations
    print 'Timewindow before check',timewindow
    if (len(sys.argv)==2):
        check_args(sys.argv)
    else:        
        while True :
            try:
                filename=raw_input('Enter data file name: ')
                if os.path.isfile(filename):
                    break
                else:
                    print 'File does not exist! Try again!'
            except SyntaxError:
                print 'Try again'
    
    fr_index=0
    #z_scale= float(5*25)/float(3600)
    z=-z_scale
    mat=sio.loadmat(filename)
    all_data=mat.get('ranges')
    angle_min=mat.get('angle_min')
    angle_max=mat.get('angle_max')
    angle_increment=mat.get('angle_increment')
    timewindow = mat.get('timewindow')
    range_limit = mat.get('range_limit')
    classifier_annotations = mat.get('annotations')
    max_index=len(all_data)
    mybuffer=all_data[0]

    limit=max_index/int(timewindow) #allocate at least 3 tw to detect wall
    print "{0} slots will be processed, after walls are removed".format(limit)
    
    sampling=np.arange(0,len(mybuffer),2)#apply sampling e.g every 2 steps
    phi=np.arange(angle_min,angle_max,angle_increment)[sampling]
    
    
    wall = mat.get('wall')#we have the wall from the mat!
    wall2 = np.zeros(len(wall)/2)
    #wall.shape(len(wall)+2,1)
    #wall = np.array(wall)
    for i in range(len(wall)-1):
        if i%2 == 0:
            wall2[i/2] += (wall[i][0])
    print wall2
    wall = wall2
    #exit()
    
    wall_cart=np.array(pol2cart(wall,phi,0) )[:,0:2] #convert to Cartesian
    kat,ax=initialize_plots(wall_cart)

    print 'Walls set...'
    
    for outer_index in range(0,max_index):

        print 'outer_index : {0}'.format(outer_index)
        raw_data=all_data[outer_index]
        raw_data=raw_data[sampling]
        filter=np.where(raw_data <= wall) #remove walls
        raw_data = raw_data[filter]
        theta =phi[filter]

        if (len(raw_data)<=3 ):
            print 'Empty scan'
            em_index=em_index+1

        if (len(raw_data)>3):
            print 'fr_index : {0}'.format(fr_index)
            print 'max_index : {0}'.format(max_index)
            z = z + z_scale
            fr_index=fr_index+1
            C=np.array(pol2cart(raw_data,theta,z) ) #convert to Cartesian

            if (fr_index % timewindow== 1 or fr_index==1):
                mybuffer=C

            if (fr_index>1) :
                mybuffer=np.concatenate((mybuffer,C), axis=0 )

            if ((fr_index % timewindow )== 0):
                mybuffer=mybuffer[np.where(mybuffer[:,0] > 0.2),:][0]
                
                print 'empty scans: {0}'.format(em_index)
                cluster_labels,human,hogs,ann,surfaces=cluster_train(mybuffer) #clustering
                print ann
                if len(hogs)!=0:
                    print'len(hogs)!=0'
                    slot_count=slot_count+1
                   # if slot_count==limit-1 :
                        #exit()
                    if slot_count>limit:
                        print 'EXITING'
                        #exit()
    if FP+TN > 0:    
        print 'Precision'
        print float(float(TP)/float(FP+TN))
    if TP+FN > 0:
        print 'Recall'
        print float(float(TP)/float(TP+FN))
    if TP+FP+TN+FN > 0:
        print 'Accuracy'
        print float(float(TP+TN)/float(TP+FP+TN+FN))
    #exit()


def pol2cart(r,theta,zed):
    x=np.multiply(r,np.cos(theta))
    y=np.multiply(r,np.sin(theta))
    z=np.ones(r.size)*zed
    C=np.array([x,y,z]).T
    return C

def initialize_plots(wall_cart):
    global fig1, fig3

    temp=plt.figure('Scan View From Above')
    plot2d = temp.add_subplot(111)
    plot2d.set_xlabel('Vertical distance')
    plot2d.set_ylabel('Robot is here')
   # plot2d.plot(wall_cart[:,0],wall_cart[:,1])

    fig1=plt.figure('3D Scan View')
    plot3d= fig1.gca(projection='3d')
    plot3d.set_xlabel('X - Distance')
    plot3d.set_ylabel('Y - Robot')
    plot3d.set_zlabel('Z - time')
    plt.show()
    
    fig3=plt.figure('3D Gridfit View')
    #plot3d= fig3.gca(projection='3d')
    #plot3d.set_xlabel('X - Distance')
    #plot3d.set_ylabel('Y - Robot')
    #plot3d.set_zlabel('Z - time')
    plt.show()
    
    return plot2d,plot3d

def cluster_train(clear_data):

    global cc, ccnames, kat, ax, fig1, wall_cart, TP, FP, TN, FN, annotations_checked, fig3
    hogs=[]
    surfaces=[]
    ann=[]

    Eps, cluster_labels= dbscan(clear_data,3) # DB SCAN
    print  len(clear_data),' points in ', np.amax(cluster_labels),'clusters'
    #print 'Eps = ', Eps, ', outliers=' ,len(np.where(cluster_labels==-1))
    max_label=int(np.amax(cluster_labels))
    human=np.zeros(len(clear_data))

    [xi,yi,zi] = [clear_data[:,0] , clear_data[:,1] , clear_data[:,2]]
    fig1.clear()
    kat.clear()
    kat.plot(wall_cart[:,0],wall_cart[:,1])

    for k in range(1,max_label+1) :
        filter=np.where(cluster_labels==k)
        if len(filter[0])>timewindow :
            ax.scatter(xi[filter],yi[filter], zi[filter], 'z', 30,c=cc[k%12])
            fig1.add_axes(ax)
            #kat.scatter(xi[filter],yi[filter],s=20, c=cc[k-1])
            kat.scatter(xi[filter],yi[filter],s=20, c=cc[k%12])
            
            
            grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            grid=grid-np.amin(grid) #build surface grid
            fig3.clear()
            ax3 = fig3.add_subplot(1,1,1, projection='3d')
            X, Y = np.mgrid[:16, :16]
            surf = ax3.plot_surface(X, Y, grid, rstride=1, cstride=1, cmap=cm.gray,
                    linewidth=0, antialiased=False)
            surfaces.append(grid)
            hogs.append(hog(grid)) #extract features
            
            plt.pause(0.0001)
            
            #print ccnames[k-1],' cluster size :',len(filter[0]), 'Is',ccnames[k-1],'human? '
            print ccnames[k%12],' cluster size :',len(filter[0]), 'Is',ccnames[k%12],'human? '
            while True:
                ha = raw_input()
                if RepresentsInt(timewindow) and (int(ha)==1 or int(ha)==0):
                    #print timewindow
                    ha = int(ha)
                    break
                else:
                    print 'Try again, 1 for human or 0 for obstacle'
                    
            if ha == classifier_annotations[annotations_checked]:
                if ha == 1:
                    TP+=1
                    print 'TP'
                    print TP
                else:
                    TN+=1
                    print 'TN'
                    print TN
            else:
                if classifier_annotations[annotations_checked] == 1:
                    FP+=1
                    print 'FP'
                    print FP
                else:
                    FN+=1
                    print 'FN'
                    print FN
            annotations_checked+=1
            human[filter]=ha
            ann.append(ha)

    return cluster_labels,human,hogs,ann,surfaces

if __name__ == '__main__':
    annotate_for_metrics()
