#!/usr/bin/env python

import  pickle
import os.path
import sys
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import time
from os import listdir
from os.path import isfile, join, splitext
from gridfit import gridfit
#from hogs_dem import hog
from skimage.feature import hog
from scipy.stats.mstats import zscore
from sklearn.decomposition import PCA
from sklearn.metrics import confusion_matrix
from sklearn import svm
from sklearn import lda
import matplotlib.pyplot as plt
np.set_printoptions(threshold='nan')


#pre-made classifiers
from sklearn import cross_validation
from sklearn.naive_bayes import GaussianNB

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

annotated_humans = 0
annotated_obstacles = 0

def RepresentsFloat(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def loadfiles(path):

    global annotations, features, points

    t_flag = True
    files=[]
    data=[]
    ann=[]

    for file in listdir(path):
    	if file.endswith("_labels.mat"):
	    files.append(file)
	    mat=sio.loadmat(path+file)

	
    files.sort()
    for file_ in files:
	    print 'FILE ',file_
	    mat=sio.loadmat(path+file_)

	    if t_flag:
		t_flag=False
	
		points=np.array((mat['point_clouds']))

	        #annotation = datamat['annotations']
	 	#for i in range(1, len(hogs))  :
		 #   features=np.vstack((features, hogs[i]))

		annotations=np.array(mat['annotations'][0])
	    else:
		#for i in range(0, len(hogs)) :
		 #   features=np.vstack((features, hogs[i]))
		annotations = np.append(annotations, mat['annotations'][0])
		points = np.vstack((points, mat['point_clouds']))
    
    #newm = sio.loadmat("cluster_labels/video1_labels.mat")

    print 'HH {} AN {}'.format(len(points),len(annotations))
    #return features, annotations



def train():

    global annotations, features, points

    features=[]
    annotations=[]
    points=[]

    path=''
    train_perc=0.7
    print "##########################################################"
    print "Run with no arguments to train with data located in current folder"
    print "To specifiy the location of the training data run as follows :"
    print ">python merge_train.py <path of folder of training data> <percentage of training set - 0.7 recommended>"
    print "##########################################################"
    if (len(sys.argv)==3):
        path = sys.argv[1]

	if RepresentsFloat(sys.argv[2]):
	    train_perc = float(sys.argv[2])
	else:
	    print 'The system will use the default percentage - 0.7'

    loadfiles(path)

    feature_extraction()
    train_set,test_set,train_ann,test_ann = create_sets(train_perc)

    print 'PCA + NB'
    NB_PCAclassification(train_set,test_set,train_ann,test_ann)
    print 'SVM'
    SVMclassification(train_set,test_set,train_ann,test_ann)
    print 'LDA'
    LDAclassification(train_set,test_set,train_ann,test_ann)

    #print 'f {} \n a {} {}'.format(features[len(features)-1], len(annotations),annotations)
    

def translate_cluster(x, y, z) :

    xmean=np.mean(x)
    ymean=np.mean(y)
    zmean=np.mean(z)

    new_x=[]
    new_y=[]
    new_z=[]

    for i in range(0, len(x)):
	new_x.append(x[i]-xmean)
	new_y.append(y[i]-ymean)
	new_z.append(z[i]-zmean)


    return [new_x,new_y,new_z]


def multiply_array(x,y,z, V) :

    new_x=[]
    new_y=[]
    new_z=[]

    for i in range(0, len(x)):
	#for j in range(0,len(V)):
	new_x.append(x[i]*V[0][0] + y[i]*V[0][1] + z[i]*V[0][2])
	new_y.append(x[i]*V[1][0] + y[i]*V[1][1] + z[i]*V[1][2])
	new_z.append(x[i]*V[2][0] + y[i]*V[2][1] + z[i]*V[2][2])

    return [new_x,new_y,new_z]


def feature_extraction():

   global points, features,fig1,ax
   global all_grids,all_align
   all_grids=[]
   all_align=[]

   '''
   fig1 = plt.figure()
   plot3d= fig1.gca(projection='3d')
   plot3d.set_xlabel('X - Distance')
   plot3d.set_ylabel('Y - Robot')
   plot3d.set_zlabel('Z - time')
   plt.show()
   '''
   

   for i in range(0,len(points)) :
	#fig1.clear()
	[xi,yi,zi]=[points[i][0][0], points[i][1][0], points[i][2][0]]

    	U,s,V=np.linalg.svd(np.cov([xi,yi,zi]), full_matrices=False)

	#translate each cluster to the begining of the axis and then do the rotation
	[xnew,ynew,znew]=translate_cluster(xi,yi,zi)

	#(traslation matrix) x (rotation matrix) = alignemt of cluster
	alignment_result=multiply_array(xnew,ynew,znew, V)
	all_align.append(alignment_result)

	grid=gridfit(alignment_result[0], alignment_result[1], alignment_result[2] , 16, 16) #extract surface - y,z,x alignment_result[1]      alignment_result[0], alignment_result[1], alignment_result[2]   xi,yi,zi

	grid=grid-np.amin(grid)
	all_grids.append(grid)

	f=hog(grid, orientations=6, pixels_per_cell=(8, 8),
                    cells_per_block=(1, 1), visualise=False)
	features.append(f)
	
	#ax.scatter(xi,yi, zi, 'z', 30, 'r') #human
	#ax.scatter(alignment_result[0],alignment_result[1], alignment_result[2], 'z', 30, 'b') #human
        #fig1.add_axes(ax)
	#fig1.show()
	


def create_sets(pos):
    global features,annotations,all_grids,all_align

    train_len = int(len(features)*pos)

    train_set=features[0:train_len:1]
    test_set=features[train_len:len(features):1]
    train_ann=annotations[0:train_len:1]
    test_ann=annotations[train_len:len(features):1]

    print 'train {} test {} train ann {} test ann {} ... '.format(len(train_set), len(test_set), len(train_ann), len(test_ann))


    return train_set, test_set, train_ann, test_ann


def NB_PCAclassification(train_set,test_set,train_ann,test_ann):

    global all_grids

    #Create z-scored data
    normalized_train = zscore(train_set)
    
    #create classifier object
    gaussian_nb=GaussianNB()
        
    #Create PCA object
    pca = PCA(n_components=20)
    pca.fit(train_set)
    normalized_train = pca.transform(train_set)

    #train the NB classifier
    gaussian_nb.fit(normalized_train, train_ann)


    #convert test data to suitable format and test the NB classifier
    normalized_test = zscore(test_set)
    test = pca.transform(test_set)
    results = gaussian_nb.predict(test)

    cm = confusion_matrix(test_ann, results)

    print 'CONFUSION MATRIX = {}'.format(cm)
    metrics(cm)


def SVMclassification(train_set,test_set,train_ann,test_ann):

    #Create z-scored data
    normalized_train = zscore(train_set)
    
        
    # Run svm classifier
    classifier = svm.SVC(kernel='rbf', gamma=0.5, C=1.0, tol=2.0)
    results = classifier.fit(train_set, train_ann).predict(test_set)
 
    cm = confusion_matrix(test_ann, results)

    print 'CONFUSION MATRIX = {}'.format(cm)
    metrics(cm)
 

def LDAclassification(train_set,test_set,train_ann,test_ann):


    #Create z-scored data
    normalized_train = zscore(train_set)

    classifier = lda.LDA('lsqr')

    #train the LDA classifier
    classifier.fit(train_set, train_ann)
        
    results = classifier.predict(test_set)
 
    cm = confusion_matrix(test_ann, results)

    print 'CONFUSION MATRIX = {}'.format(cm)
    metrics(cm)
 

def metrics(cm):

   TP=float(cm[0][0])
   FP=float(cm[0][1])
   FN=float(cm[1][0])
   TN=float(cm[1][1])
    
   precision_obstr=(TP/(TP+FP))*100
   precision_hum=(TN/(TN+FN))*100
   recall_obstr=(TP/(TP+FN))*100
   recall_hum=(TN/(TN+FP))*100
   accuracy=((TP+TN)/(TP+TN+FP+FN) )*100

   print 'precision {} % recall {} % accuracy {} %'.format(precision_hum,recall_hum,accuracy)



if __name__ == '__main__':
    train()
