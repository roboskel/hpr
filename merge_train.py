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
from scipy.stats.mstats import zscore
from sklearn.decomposition import PCA
np.set_printoptions(threshold='nan')


#pre-made classifiers
from sklearn import cross_validation
from sklearn.naive_bayes import GaussianNB

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

annotated_humans = 0
annotated_obstacles = 0

def DisplayClassifier():
    t_files_set = False
    path=''
    print "##########################################################"
    print "Run with no arguments to train with data located in current folder"
    print "To specifiy the location of the training data run as follows :"
    print ">python merge_train.py <path of folder of training data>"
    print "##########################################################"
    if (len(sys.argv)==2):
        path = sys.argv[1]
    try:
        os.remove(path+"traindata_merged.p")
    except OSError:
        pass
    try:
        os.remove(path+"annotations_merged.p")
    except OSError:
        pass
    try:
        os.remove(path+"Gaussian_NB_classifier_merged.p")
    except OSError:
        pass
    try:
        os.remove(path+"PCA_object.p")
    except OSError:
        pass
    
    onlyfiles = [ f for f in listdir(path) if isfile(join(path,f)) ]
    onlyfiles.sort()
    #print onlyfiles
    ann_files = []
    t_files = []
    for file_ in onlyfiles:
        ext = os.path.splitext(file_)
        if ext[1] == '.p' and (ext[0].find('annotations')!=-1):
            ann_files += list(pickle.load( open( path+file_, "rb" ) ))
        elif ext[1] == '.p' and (ext[0].find('traindata')!=-1):
            if t_files_set :
                t_files = np.vstack((t_files, pickle.load( open( path+file_, "rb" ) )))
            else:
                t_files_set = True
                t_files = pickle.load( open( path+file_, "rb" ) )
    pickle.dump(t_files, open(path+"traindata_merged.p","wb+"))
    pickle.dump(ann_files, open(path+"annotations_merged.p","wb+"))
    annot = {}
    annot["annotations"] = ann_files
    np_ann_files = np.array(ann_files)
    np_ann_files = np.sort(np_ann_files)
    counts = np.bincount(np_ann_files)
    annot["humans"] = counts[1]
    annot["obstacles"] = counts[0]
    sio.savemat('full_set_annotations', annot);
    
    #Create z-scored data
    temp = zscore(t_files)
    
    #create classifier object
    gaussian_nb=GaussianNB()
        
    #Create PCA object
    pca = PCA()
    pca.fit(temp)
    temp = pca.transform(temp)
    gaussian_nb.fit(temp, ann_files)
    
    pickle.dump( gaussian_nb, open(path+"Gaussian_NB_classifier_merged.p", "wb+" ) )
    pickle.dump( pca, open(path+"PCA_object.p", "wb+"))
    
    raw_input("Press any key to exit")


if __name__ == '__main__':
    DisplayClassifier()
