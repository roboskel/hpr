#!/usr/bin/env python
__author__="athanasia sapountzi"

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
from numpy import loadtxt
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
timewindow=40
filename=''
wall_end = 0
range_limit = 0

annotated_humans = 0
annotated_obstacles = 0

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
    global timewindow, wall_end, filename
    print "Arguments given : {0}".format(str(sys.argv))
    timewindow = int(arg_list[1])
    if not RepresentsInt(arg_list[1]):
        while True:
            timewindow = raw_input('Set timewindow in frames: ')
            if RepresentsInt(timewindow):
                #print timewindow
                timewindow = int(timewindow)
                break
            else:
                print 'Try again'
        
    
    wall_end = int(arg_list[2])
    if not RepresentsInt(wall_end):
        while True:
            wall_end = raw_input('Set max frames for wall setting: ')
            if RepresentsInt(wall_end):
                wall_end = int(wall_end)
                break
            else:
                print 'Try again'
                    
    filename = str(arg_list[3])
    
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
    print "Wall frames : {0}".format(wall_end)
    print 'Timewindow : {0}'.format(timewindow)
            
def offline_train():
    print "#######################"
    print "Non interactive train :"
    print "python annotate.py <timewindow> <wall_frames> <data_file_name>"
    print "You gave {0} arguments".format(len(sys.argv))
    print "#######################"
    
    global fr_index , ind ,all_hogs, all_surf,fig1,ax,kat,wall_cart,wall,kat,mat_file
    global z, z_scale, slot_count, human_l ,cluster_l,slot_touched,all_scans
    global timewindow ,slot_data , phi,wall_index,annotations,em_index, filename, wall_end, range_limit, point_clouds

    point_clouds=[]


    if (len(sys.argv)==4):
        #FULL ARGUMENTS GIVEN
        check_args(sys.argv)
        mat = sio.loadmat(filename)
        all_data = mat.get('ranges')
        range_limit = np.max(all_data)
        print "Maximum scan Range : {0}".format(range_limit)
        angle_min = mat.get('angle_min')
        angle_max = mat.get('angle_max')
        angle_increment = mat.get('angle_increment')
        max_index = len(all_data)
    elif (len(sys.argv)==2):
        print "READING LEGACY .TXT"
        #READ ONLY LEGACY TXT
        filename = sys.argv[1]
        while not os.path.isfile(filename) :
            try:
                filename = raw_input('Enter data file name: ')
                if os.path.isfile(filename):
                    break
                else:
                    print 'File does not exist! Try again!'
            except SyntaxError:
                print 'Try again'
        #lines = loadtxt(filename, comments="#", delimiter=" ", unpack=False)
        test_array=[]
        with open(filename) as f:
            for line in f:
                numbers_float = map(float, line.split())
                test_array.append(numbers_float)
        z_scale= float(5*40) / float(3600)
        z=-z_scale
        all_data = test_array
        range_limit = np.max(all_data)
        angle_min = -1.74532925
        angle_max = 1.74532925
        angle_increment = 0.00872664626
        max_index = len(all_data)
        print "READ .TXT"
    else:
        while True:
            timewindow=raw_input('Set timewindow in frames: ')
            if RepresentsInt(timewindow):
                timewindow=int(timewindow)
                break
            else:
                print 'Try again'
        while True:
            wall_end=input('Set max frames for wall setting: ')
            if RepresentsInt(wall_end):
                break
            else:
                print 'Try again'
        while True :
            try:
                filename=raw_input('Enter data file name: ')
                if os.path.isfile(filename):
                    mat = sio.loadmat(filename)
                    all_data = mat.get('ranges')
                    range_limit = np.max(all_data)
                    angle_min = mat.get('angle_min')
                    angle_max = mat.get('angle_max')
                    angle_increment = mat.get('angle_increment')
                    max_index = len(all_data)
                    break
                else:
                    print 'File does not exist! Try again!'
            except SyntaxError:
                print 'Try again'
    
    fr_index=0
    #z_scale= float(5*25)/float(3600)
    z=-z_scale
    
    mybuffer=all_data[0]

    #read_clouds()
    #exit()

    #limit=((max_index-wall_end-120)/timewindow) #epitrepo 3 s kena
    limit=(max_index-wall_end-(3*int(timewindow)))/int(timewindow) #allocate at least 3 tw to detect wall
    print "{0} slots will be processed, after walls are removed".format(limit)
    
    #print 'Reduce points by 2? 1/0'
    #if input()==1 :
    
    sampling=np.arange(0,len(mybuffer),2)#apply sampling e.g every 2 steps
    
    #else :
       #sampling=np.arange(0,len(mybuffer),1)
    #sort scans
    
    phi=np.arange(angle_min,angle_max,angle_increment)[sampling]
    wall=all_data[0]

    for wall_index in range(1,wall_end):

        print 'Wall_index : {0}'.format(wall_index)
        #wall=all_data[0]
        filter=np.where(wall>=range_limit)
        wall[filter]=range_limit

        if (wall_index<wall_end):
            mybuffer=np.vstack((mybuffer,wall ))  #  add to buffer with size=(wall_index x 720)

    mybuffer=mybuffer[:,sampling]

    wall=np.min(mybuffer, axis=0)-0.1 #select min of measurements
    print "Wall index : {0}".format(wall_index)
    print "Wall : {0}".format(wall)
    wall_cart=np.array(pol2cart(wall,phi,0) )[:,0:2] #convert to Cartesian
    kat,ax=initialize_plots(wall_cart)

    print 'Walls set...'
    
    for outer_index in range(wall_index,max_index):

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
                #print 'Buffer1'
                mybuffer=C

            if (fr_index>1) :
                #print 'Buffer2'
                mybuffer=np.concatenate((mybuffer,C), axis=0 )
                #print 'Buffer2b'

            if ((fr_index % timewindow )== 0):
                #print'Buffer3'
                mybuffer=mybuffer[np.where(mybuffer[:,0] > 0.2),:][0]
                
                print 'empty scans: {0}'.format(em_index)
                cluster_labels,human,hogs,ann,surfaces,cluster_points = cluster_train(mybuffer) #clustering

                if len(hogs)!=0:
                    print'len(hogs)!=0'
                    slot_count=slot_count+1
                    print 'File : {0}'.format(filename)
                    print 'slot count : {0} || limit :{1}'.format(slot_count, limit)
                    ha=np.array(slot_count*np.ones(len(mybuffer))) #data point -> slot_number

                    if slot_count==1:
                            slot_data=ha
                            all_scans=mybuffer
                            human_l=human
                            cluster_l=cluster_labels
                            all_surf=surfaces
                            all_hogs=hogs
			    #point_clouds.append(cluster_points)
                            annotations=ann

                    else:
                            all_surf=np.vstack((all_surf,surfaces))
                            all_hogs=np.vstack((all_hogs,hogs))
			    #point_clouds.append(cluster_points)
                            slot_data=np.hstack((slot_data,ha))
                            all_scans=np.vstack((all_scans,mybuffer) )
                            cluster_l=np.hstack((cluster_l,cluster_labels))
                            human_l=np.hstack((human_l,human))
                            annotations=np.hstack((annotations,ann))
                    
                    if slot_count==limit-1 :
                        build_classifier(np.array(all_hogs),np.array(annotations))
                        save_data()
                        exit()
                    if slot_count>limit:
                        print 'EXITING'
                        exit()
        
    #print 'all hogs {} point clouds {}'.format(all_hogs, point_clouds)         
    build_classifier(np.array(all_hogs),np.array(annotations))
    save_clouds(np.array(annotations), point_clouds)
    save_data()
    #exit()


def read_clouds():

    mat_content=sio.loadmat(filename[:-4]+'_labels')

    return mat_content['point_clouds'], mat_content['annotation'][0]




def save_clouds(annotations, point_clouds):

    b={}
    b['annotations']=annotations
    b['point_clouds']=point_clouds

    sio.savemat(filename[:-4]+'_labels',b);
    


def pol2cart(r,theta,zed):
    x=np.multiply(r,np.cos(theta))
    y=np.multiply(r,np.sin(theta))
    z=np.ones(r.size)*zed
    C=np.array([x,y,z]).T
    return C

def initialize_plots(wall_cart):
    global fig1, fig3

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
    
    fig3=plt.figure('3D Gridfit View')
    #plot3d= fig3.gca(projection='3d')
    #plot3d.set_xlabel('X - Distance')
    #plot3d.set_ylabel('Y - Robot')
    #plot3d.set_zlabel('Z - time')
    plt.show()
    
    return plot2d,plot3d


def euclidean_distance(v1, v2):
    '''
    list_dist = []

    min_dist = -1.0
    index=0

    
    if isinstance(v1, list):
        for i in range(0,len(v1)):
	    distance = dist.euclidean(np.array(v1)[i],v2)
	        #print distance
	    if min_dist == -1.0:
		min_dist=distance
		index=j
	    elif distance <= min_dist:
		min_dist=distance
		index=j

	 list_dist.append(index)
	 index=0
	 min_dist=-1.0
    
    if not isinstance(v1, list):
	#print 'v1 has a single hog array'
	for j in range(0,len(v2)):
	    distance = dist.euclidean(v1,np.array(v2)[j])
	    #print distance
	    if min_dist == -1.0:
		min_dist=distance
		index=j
	    elif distance <= min_dist:
		min_dist=distance
		index=j

	list_dist.append(index)
    if flag==True:
	#print 'v2 has a single hog array'
	for j in range(0,len(v1)):
	    distance = dist.euclidean(np.array(v1)[j],v2)
	    #print distance
	    if min_dist == -1.0:
		min_dist=distance
		index=j
	    elif distance <= min_dist:
		min_dist=distance
		index=j

	list_dist.append(index)
    
    return list_dist
    '''


def cluster_train(clear_data):

    global cc, ccnames, kat, ax, fig1, wall_cart, fig3, hogs_temp
    global annotated_humans, annotated_obstacles, cc, point_clouds
    hogs=[]
    surfaces=[]
    ann=[]
    cluster_points=[]

    Eps, cluster_labels= dbscan(clear_data,3) # DB SCAN
    print  'eps = ',Eps,' , ',len(clear_data),' points in ', np.amax(cluster_labels),'clusters'
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

	    [xk,yk,zk] = [xi[filter],yi[filter], zi[filter]]
	    point_clouds.append([xk,yk,zk])
            
            grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            grid=grid-np.amin(grid) #build surface grid
            fig3.clear()
            ax3 = fig3.add_subplot(1,1,1, projection='3d')
            X, Y = np.mgrid[:16, :16]
            surf = ax3.plot_surface(X, Y, grid, rstride=1, cstride=1, cmap=cm.gray,
                    linewidth=0, antialiased=False)
            surfaces.append(grid)
            hogs.append(hog(grid)) #extract features

	    #list_dist=euclidean_distance(hogs_temp, hog(grid))
            
            plt.pause(0.0001)
            
            #print ccnames[k-1],' cluster size :',len(filter[0]), 'Is',ccnames[k-1],'human? '
            print ccnames[k%12],' cluster size :',len(filter[0]), 'Is',ccnames[k%12],'human? '
            while True:
                ha = raw_input()
                if RepresentsInt(timewindow) and (int(ha)==1 or int(ha)==0):
                    #print timewindow
                    ha = int(ha)
                    if ha == 1:
                        annotated_humans = annotated_humans + 1
                    else :
                        annotated_obstacles = annotated_obstacles + 1
                    break
                else:
                    print 'Try again, 1 for human or 0 for obstacle'
                    
            #grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            #grid=grid-np.amin(grid) #build surface grid
            #surfaces.append(grid)
            #hogs.append(hog(grid)) #extract features
            human[filter]=ha
            ann.append(ha)

    hogs_temp = np.array(np.array(hogs))
    return cluster_labels,human,hogs,ann,surfaces,cluster_points


def build_classifier(traindata, annotations):
    global timewindow
    #------------   BUILD CLASSIFIERS -------------
    
    
    print 'Preparing classifiers ...'
    pickle.dump(wall, open(filename.replace(' ', '')[:-4]+"wall.p","wb+"))
    pickle.dump(traindata, open(filename.replace(' ', '')[:-4]+"traindata.p","wb+"))
    pickle.dump(annotations, open(filename.replace(' ', '')[:-4]+"annotations.p","wb+"))
    pickle.dump(timewindow, open(filename.replace(' ', '')[:-4]+"timewindow.p","wb+"))
    #Apply PCA z score
    temp=zscore(traindata)
    pickle.dump(temp , open( filename.replace(' ', '')[:-4]+"z_score.p", "wb+" ))
    gaussian_nb = GaussianNB()
    gaussian_nb.fit(temp,annotations)
    pickle.dump( gaussian_nb, open( filename.replace(' ', '')[:-4]+"Gaussian_NB_classifier.p", "wb+" ) )
    ''''
    if os.path.exists("Gaussian_NB_classifier.p"):
        gaussian_nb = pickle.load( open( "Gaussian_NB_classifier.p", "rb" ) )
        gaussian_nb.fit(temp,annotations)
        pickle.dump( gaussian_nb, open( "Gaussian_NB_classifier.p", "wb+" ) )
        pickle.dump( gaussian_nb, open( filename.replace(' ', '')[:-4]+"Gaussian_NB_classifier.p", "wb+" ) )
    else:
        gaussian_nb=GaussianNB()
        gaussian_nb.fit(temp,annotations)
        pickle.dump( gaussian_nb, open( "Gaussian_NB_classifier.p", "wb+" ) )
        pickle.dump( gaussian_nb, open( filename.replace(' ', '')[:-4]+"Gaussian_NB_classifier.p", "wb+" ) )
    '''    

def save_data():
    global wall,slot_data,all_scans,cluster_l,timewindow,phi,all_hogs
    global annotated_humans, annotated_obstacles
        #------------   SAVE DATA ----------------

    print 'Saving data ...'
    b={}
    b['wall_ranges']=wall
    b['timewindow']=slot_data #slot number of each point
    b['cluster_cartesians']=all_scans #cartesian coordinates of cluster points
    b['human_labels']=human_l #annotations of cluster points
    b['cluster_labels']=cluster_l
    b['timewindow']=timewindow
    b['rad_angles']=phi
    b['hogs']=all_hogs
    b['surfaces']=all_surf
    b['number_of_humans']=annotated_humans
    b['number_of_obstacles']=annotated_obstacles
    #sio.savemat('training_data',b)
    sio.savemat(filename[:-4]+'_training',b);
    print 'done saving'
    print "Annotated Humans : {0}".format(annotated_humans)
    print "Annotated Obstacles : {0}".format(annotated_obstacles)

if __name__ == '__main__':
    offline_train()
