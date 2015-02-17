#!/usr/bin/env python
__author__="athanasia sapountzi"

import  pickle
from gridfit import gridfit
import numpy as np
import scipy.io as sio
from scipy import special
import matplotlib.pyplot as plt
from mytools import princomp,dbscan
from myhog import hog
from scipy.stats.mstats import zscore
import os.path

#pre-made classifiers
from sklearn.naive_bayes import GaussianNB

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

ccnames =['GRAY', 'BLACK', 'VIOLET', 'BLUE', 'CYAN', 'ROSY', 'ORANGE', 'RED', 'GREEN', 'BROWN', 'YELLOW', 'GOLD']
cc  =  ['#808080',  'k',  '#990099', '#0000FF', 'c','#FF9999','#FF6600','r','g','#8B4513','y','#FFD700']
#ccnames =['BLUE', 'RED']
#cc  =  ['#0000FF', 'r']


fr_index=0
z_scale= float(5*40) / float(3600)
z=-z_scale
slot_count=0

slot_touched=0

flag=0
plt.ion()
wall_flag=0
wall_index=1
em_index=0
timewindow=40
filename='asd'
#mat_file='mat_files/'

def offline_train():

    global fr_index , ind ,all_hogs, all_surf,fig1,ax,kat,wall_cart,wall,kat,mat_file
    global z, zscale, slot_count, human_l ,cluster_l,slot_touched,all_scans
    global timewindow ,slot_data , phi,wall_index,annotations,em_index, filename
    
    timewindow=input('Set timewindow in frames: ')
    wall_end=input('Set max frames for wall setting: ')
    range_limit=input('Set maximum scan range: ')
    filename=input('Enter data file name: ')
    #mat=sio.loadmat('bagfile_data.mat')
    mat=sio.loadmat(filename)
    #mat=sio.loadmat(prefix+filename+'.mat')
    all_data=mat.get('ranges')
    #all_data=all_data.T
    #print all_data
    angle_min=mat.get('angle_min')
    angle_max=mat.get('angle_max')
    angle_increment=mat.get('angle_increment')
    max_index=len(all_data)
    mybuffer=all_data[0]

    limit=((max_index-wall_end-120)/timewindow) #epitrepo 3 s kena

    print limit, 'Slots will be processed, after walls'
    
    #print 'Reduce points by 2? 1/0'
    #if input()==1 :
    sampling=np.arange(0,len(mybuffer),2)#apply sampling e.g every 2 steps
   # else :
       #  sampling=np.arange(0,len(mybuffer),1)
    
    phi=np.arange(angle_min,angle_max,angle_increment)[sampling]
    wall=all_data[0]

    for wall_index in range(1,wall_end):

        print 'wall_index'
        #wall=all_data[0]
        filter=np.where(wall>=range_limit)
        wall[filter]=range_limit

        if (wall_index<wall_end):
            mybuffer=np.vstack((mybuffer,wall ))  #  add to buffer with size=(wall_index x 720)

    mybuffer=mybuffer[:,sampling]

    wall=np.min(mybuffer, axis=0)-0.1 #select min of measurements
    print wall_index
    wall_cart=np.array(pol2cart(wall,phi,0) )[:,0:2] #convert to Cartesian
    kat,ax=initialize_plots(wall_cart)

    print 'Walls set...'
    
    for outer_index in range(wall_index,max_index):

        print 'outer_index'
        raw_data=all_data[outer_index]
        raw_data=raw_data[sampling]
        filter=np.where(raw_data <= wall) #remove walls
        raw_data = raw_data[filter]
        theta =phi[filter]

        if (len(raw_data)<=3 ):
            print 'empty scan'
            em_index=em_index+1

        if (len(raw_data)>3):
            print 'fr_index '
            print  fr_index
            print 'max_index'
            print max_index
            z = z + z_scale
            fr_index=fr_index+1
            C=np.array(pol2cart(raw_data,theta,z) ) #convert to Cartesian

            if (fr_index % timewindow== 1 or fr_index==1):
                print 'Buffer1'
                mybuffer=C

            if (fr_index>1) :
                print 'Buffer2'
                mybuffer=np.concatenate((mybuffer,C), axis=0 )
                print 'Buffer2b'

            if ((fr_index % timewindow )== 0):
                print'Buffer3'
                mybuffer=mybuffer[np.where(mybuffer[:,0] > 0.2),:][0]
                print 'slot count : ',slot_count
                print 'empty scans: ',em_index
                cluster_labels,human,hogs,ann,surfaces=cluster_train(mybuffer) #clustering

                if len(hogs)!=0:
                    print'len(hogs)!=0'
                    slot_count=slot_count+1
                    ha=np.array(slot_count*np.ones(len(mybuffer))) #data point -> slot_number

                    if slot_count==1:
                            slot_data=ha
                            all_scans=mybuffer
                            human_l=human
                            cluster_l=cluster_labels
                            all_surf=surfaces
                            all_hogs=hogs
                            annotations=ann

                    else:
                            all_surf=np.vstack((all_surf,surfaces))
                            all_hogs=np.vstack((all_hogs,hogs))
                            slot_data=np.hstack((slot_data,ha))
                            all_scans=np.vstack((all_scans,mybuffer) )
                            cluster_l=np.hstack((cluster_l,cluster_labels))
                            human_l=np.hstack((human_l,human))
                            annotations=np.hstack((annotations,ann))
                    
                    if slot_count==limit-1 :
                        build_classifier(np.array(all_hogs),np.array(annotations))
                        save_data()
                    if slot_count>limit:
                        print 'EXITING'
                        exit()
                          
                          
    build_classifier(np.array(all_hogs),np.array(annotations))
    save_data()
    #exit()


def pol2cart(r,theta,zed):
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

def cluster_train(clear_data):

    global cc, ccnames, kat, ax, fig1, wall_cart
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
            plt.pause(0.0001)
            #print ccnames[k-1],' cluster size :',len(filter[0]), 'Is',ccnames[k-1],'human? '
            print ccnames[k%12],' cluster size :',len(filter[0]), 'Is',ccnames[k%12],'human? '
            while True :
                try:
                    ha=input()
                    break
                except SyntaxError:
                    print 'try again'
                    print ccnames[k%12],' cluster size :',len(filter[0]), 'Is',ccnames[k%12],'human? '

            grid=gridfit(yi[filter], zi[filter], xi[filter], 16, 16) #extract surface
            grid=grid-np.amin(grid) #build surface grid
            surfaces.append(grid)
            hogs.append(hog(grid)) #extract features
            human[filter]=ha
            ann.append(ha)

    return cluster_labels,human,hogs,ann,surfaces


def build_classifier(traindata, annotations):
    global timewindow
	#------------   BUILD CLASSIFIERS -------------
    '''
    print 'preparing classifiers ...'
    pickle.dump(wall, open("wall.p","wb+"))
    pickle.dump(traindata, open("traindata.p","wb+"))
    pickle.dump(annotations, open("annotations.p","wb+"))
    pickle.dump(timewindow, open("timewindow.p","wb+"))
    #Apply PCA z score
    temp=zscore(traindata)
    pickle.dump(temp , open( "z_score.p", "wb+" ))
    gaussian_nb=GaussianNB()
    gaussian_nb.fit(temp,annotations)
    pickle.dump( gaussian_nb, open( "Gaussian_NB_classifier.p", "wb+" ) )
    '''
    print 'preparing classifiers ...'
    pickle.dump(wall, open(filename.replace(' ', '')[:-4]+"wall.p","wb+"))
    pickle.dump(traindata, open(filename.replace(' ', '')[:-4]+"traindata.p","wb+"))
    pickle.dump(annotations, open(filename.replace(' ', '')[:-4]+"annotations.p","wb+"))
    pickle.dump(timewindow, open(filename.replace(' ', '')[:-4]+"timewindow.p","wb+"))
    #Apply PCA z score
    temp=zscore(traindata)
    pickle.dump(temp , open( filename.replace(' ', '')[:-4]+"z_score.p", "wb+" ))
    #exit()
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
		

def save_data():
    global wall,slot_data,all_scans,cluster_l,timewindow,phi,all_hogs

        #------------   SAVE DATA ----------------

    print 'saving data ...'
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
    sio.savemat('training_data',b)
    print 'done saving'


if __name__ == '__main__':
    offline_train()
