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

def loadfiles():

    global path

    annotations=[]
    points=[]

    files=[]

    for file in listdir(path):
    	if file.endswith("_labels.mat"):
	    files.append(file)
	    mat=sio.loadmat(path+file)

	
    files.sort()

    return files



def load_data(files):
    t_flag = True

    for file_ in files:
	    print 'FILE ',file_
	    mat=sio.loadmat(path+file_)

	    if t_flag:
		t_flag=False
	
		points=np.array((mat['point_clouds']))

		annotations=np.array(mat['annotations'][0])
	    else:
		points = np.vstack((points, mat['point_clouds']))
		annotations = np.append(annotations, mat['annotations'][0])
    
    #newm = sio.loadmat("cluster_labels/video1_labels.mat")

    print 'Num of points {} Num of annotations {}'.format(len(points),len(annotations))
    return points, annotations



def train():

    global path, train_perc

    path=''
    train_perc=0.7

    print "##########################################################"
    print "Run with no arguments to train with data located in current folder"
    print "To specifiy the location of the training data run as follows :"
    print ">python recognition_procedure.py <path of folder of training data> <percentage of training set - 0.7 recommended - or 1.0 for cross validation>"
    print "##########################################################"
    if (len(sys.argv)==3):
        path = sys.argv[1]

	if RepresentsFloat(sys.argv[2]):
	    train_perc = float(sys.argv[2])
	    if train_perc <= 0.0 or train_perc > 1.0:
		print 'the second parameter can only take values from the range (0.0 , 1.0]. Please try again...'
		exit()
	else:
	    print 'The system will use the default percentage - 0.7'

    video_list = loadfiles()


    if train_perc < 1.0:
	points, annotations = load_data(video_list)
	features = feature_extraction(points)
    	train_set,test_set,train_ann,test_ann = create_sets(train_perc, features, annotations)

	print 'PCA + NB'
    	NB_PCAclassification(train_set,test_set,train_ann,test_ann)
    	print 'SVM'
    	SVMclassification(train_set,test_set,train_ann,test_ann)
    	print 'LDA'
    	LDAclassification(train_set,test_set,train_ann,test_ann)
    elif train_perc == 1.0:	#cross validation
	video_cross_validation(video_list)
    
    
    
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
	new_x.append(x[i]*V[0][0] + y[i]*V[0][1] + z[i]*V[0][2])
	new_y.append(x[i]*V[1][0] + y[i]*V[1][1] + z[i]*V[1][2])
	new_z.append(x[i]*V[2][0] + y[i]*V[2][1] + z[i]*V[2][2])

    return [new_x,new_y,new_z]


def feature_extraction(points):

    global fig1,ax
    global all_grids,all_align
    all_grids=[]
    all_align=[]
    features = []

   
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
	
    return features



def create_sets(pos, features, annotations):

    train_len = int(len(features)*pos)

    train_set=features[0:train_len:1]
    test_set=features[train_len:len(features):1]
    train_ann=annotations[0:train_len:1]
    test_ann=annotations[train_len:len(features):1]

    #print 'train {} test {} train ann {} test ann {} ... '.format(len(train_set), len(test_set), len(train_ann), len(test_ann))


    return train_set, test_set, train_ann, test_ann


def video_cross_validation(video_list):
    global NB_accuracy_array, NB_precision_array, NB_recall_array
    global SVM_accuracy_array, SVM_precision_array, SVM_recall_array
    global LDA_accuracy_array, LDA_precision_array, LDA_recall_array

    NB_accuracy_array = []
    NB_precision_array = []
    NB_recall_array = []

    SVM_accuracy_array = []
    SVM_precision_array = []
    SVM_recall_array = []

    LDA_accuracy_array = []
    LDA_precision_array = []
    LDA_recall_array = []


    for i in range(0,len(video_list)) :
	test = video_list[i]
	train = video_list[0:i] + video_list[i+1:]

	print '\n-------------------------------------------------------------Round ',i,':'
	cross_validation(train, test)
	

    NB_acc = float(sum(NB_accuracy_array) / len(NB_accuracy_array))
    #SVM_acc = float(sum(SVM_accuracy_array) / len(SVM_accuracy_array))
    LDA_acc = float(sum(LDA_accuracy_array) / len(LDA_accuracy_array))

    NB_prec = float(sum(NB_precision_array) / len(NB_precision_array))
    LDA_prec = float(sum(LDA_precision_array) / len(LDA_precision_array))

    NB_rec = float(sum(NB_recall_array) / len(NB_recall_array))
    LDA_rec = float(sum(LDA_recall_array) / len(LDA_recall_array))

    print 'Total Results: \nNB accuracy = {} % NB precision {} % NB recall {} % \n LDA accuracy = {} % LDA precision {} % LDA recall {} % '.format(NB_acc, NB_prec, NB_rec, LDA_acc, LDA_prec, LDA_rec)


def cross_validation(train, test):
    global path
    global NB_accuracy_array, NB_precision_array, NB_recall_array
    global SVM_accuracy_array, SVM_precision_array, SVM_recall_array
    global LDA_accuracy_array, LDA_precision_array, LDA_recall_array

    train_points = []
    test_points = []
    train_annotations = []
    test_annotations = []
    t_flag = True
    
    #print 'video test {} , videos train {}'.format(test, train)

    for file_ in train:
    	mat=sio.loadmat(path+file_)

	if t_flag:
	    t_flag=False
	
	    train_points=np.array((mat['point_clouds']))

	    train_annotations=np.array(mat['annotations'][0])
	else:
	    train_annotations = np.append(train_annotations, mat['annotations'][0])
	    train_points = np.vstack((train_points, mat['point_clouds']))
    
    train_features = feature_extraction(train_points)	#get features of the train set

    #get annotations, points and extract features for the test set 
    mat = sio.loadmat(path+test)
    test_annotations=np.array(mat['annotations'][0])
    test_points = np.array((mat['point_clouds']))
    
    test_features = feature_extraction(test_points)

    print 'PCA + NB'
    acc,prec,rec = NB_PCAclassification(train_features, test_features, train_annotations, test_annotations)
    NB_accuracy_array.append(acc)
    NB_precision_array.append(prec)
    NB_recall_array.append(rec)

    '''
    acc,prec,rec = SVMclassification(train_features, test_features, train_annotations, test_annotations)
    SVM_accuracy_array.append(acc)
    SVM_precision_array.append(prec)
    SVM_recall_array.append(rec)
    '''
   
    print 'LDA'
    acc,prec,rec = LDAclassification(train_features, test_features, train_annotations, test_annotations)
    LDA_accuracy_array.append(acc)
    LDA_precision_array.append(prec)
    LDA_recall_array.append(rec)


def NB_PCAclassification(train_set,test_set,train_ann,test_ann):

    global path

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


    #store the classifier and the pca object
    #pickle.dump( gaussian_nb, open(path+"GaussianNB_classifier.p", "wb+" ) )
    #pickle.dump( pca, open(path+"PCA_object.p", "wb+"))

    #convert test data to suitable format and test the NB classifier
    normalized_test = zscore(test_set)
    test = pca.transform(test_set)
    results = gaussian_nb.predict(test)

    cm = confusion_matrix(test_ann, results)

    print 'CONFUSION MATRIX = {}'.format(cm)
    return metrics(cm)


def SVMclassification(train_set,test_set,train_ann,test_ann):
  
    global path

    #Create z-scored data
    normalized_train = zscore(train_set)
    
        
    # Run svm classifier
    classifier = svm.SVC(kernel='rbf', gamma=0.5, C=1.0, tol=2.0)
    results = classifier.fit(train_set, train_ann).predict(test_set)
 
    #store the trained classifier
    #pickle.dump( classifier, open(path+"SVM_classifier.p", "wb+" ) )

    cm = confusion_matrix(test_ann, results)

    print 'CONFUSION MATRIX = {}'.format(cm)
    return metrics(cm)
 

def LDAclassification(train_set,test_set,train_ann,test_ann):

    global path

    #Create z-scored data
    normalized_train = zscore(train_set)

    classifier = lda.LDA('lsqr')

    #train the LDA classifier
    classifier.fit(train_set, train_ann)

    #store the trained classifier
    #pickle.dump( classifier, open(path+"LDA_classifier.p", "wb+" ) )
        
    results = classifier.predict(test_set)
 
    #res2 = classifier.predict()


    cm = confusion_matrix(test_ann, results)

    print 'CONFUSION MATRIX = {}'.format(cm)
    return metrics(cm)
 

def metrics(cm):

   TP=float(cm[0][0])
   FP=float(cm[0][1])
   FN=float(cm[1][0])
   TN=float(cm[1][1])
    
   try:
   	precision_obstr=(TP/(TP+FP))*100
   except ZeroDivisionError:
	precision_obstr = 0.0

   try:
   	precision_hum=(TN/(TN+FN))*100
   except ZeroDivisionError:
	precision_hum = 0.0

   try:
   	recall_obstr=(TP/(TP+FN))*100
   except ZeroDivisionError:
	recall_obstr = 0.0

   try:
   	recall_hum=(TN/(TN+FP))*100
   except ZeroDivisionError:
	recall_hum = 0.0

   try:
   	accuracy=((TP+TN)/(TP+TN+FP+FN) )*100
   except ZeroDivisionError:
	accuracy = 0.0

   print 'precision {} % recall {} % accuracy {} %'.format(precision_hum,recall_hum,accuracy)

   return accuracy, precision_hum, recall_hum



if __name__ == '__main__':
    train()
