import  pickle
#import os.path
import sys
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import time
import os
from os import listdir
from os.path import isfile, join, splitext
from mytools import princomp,dbscan
from myhog import hog
from scipy import special
from scipy.stats.mstats import zscore
from gridfit import gridfit
from sklearn.naive_bayes import GaussianNB

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
    #global timewindow, range_limit, wall_end, filename
    #print (arg_list)
    if len(sys.argv)!=2:
        print 'Nope'
        exit()
    else:
        path = str(arg_list[1])
        if not os.path.isdir(path):
            while True :
                try:
                    path=raw_input('Enter folder containing .p files: ')
                    if os.path.isdir(path):
                        break
                    else:
                        print 'Nope'
                except SyntaxError:
                    print 'Try again'
    return path
    
def UpdateClassifier():
    
    list = sys.argv
    path=check_args(list)
    onlyfiles = [ f for f in listdir(path) if isfile(join(path,f)) ]
    mat_files = []
    p_files = []
     
    for file_ in onlyfiles:
        ext = os.path.splitext(file_)
        if ext[1] == '.mat' and (ext[0].find('training')==-1):
            print file_
            mat_files.append(ext[0])
        else :
            pass
            #p_files.append(file_)
    print mat_files
    print len(mat_files)
    #print len(p_files)
    for mfile in mat_files:
        
        print  mfile+'wall.p'
        wall = pickle.load(open( path+'/'+mfile+'wall.p', "rb" ) )
        
        print  mfile+'traindata.p'
        traindata = pickle.load(open( path+'/'+mfile+'traindata.p', "rb" ) )
        
        print  mfile+'annotations.p'
        annotations = pickle.load(open( path+'/'+mfile+'annotations.p', "rb" ) )
        
        print  mfile+'timewindow.p'
        timewindow = pickle.load(open( path+'/'+mfile+'timewindow.p', "rb" ) )
        
        temp=zscore(traindata)
        
        if os.path.exists("Gaussian_NB_classifier.p"):
            gaussian_nb = pickle.load( open( "Gaussian_NB_classifier.p", "rb" ) )
            gaussian_nb.fit(temp,annotations)
            pickle.dump( gaussian_nb, open( "Gaussian_NB_classifier.p", "wb+" ) )
            #pickle.dump( gaussian_nb, open( filename.replace(' ', '')[:-4]+"Gaussian_NB_classifier.p", "wb+" ) )
        else:
            gaussian_nb=GaussianNB()
            gaussian_nb.fit(temp,annotations)
            pickle.dump( gaussian_nb, open( "Gaussian_NB_classifier.p", "wb+" ) )
            #pickle.dump( gaussian_nb, open( filename.replace(' ', '')[:-4]+"Gaussian_NB_classifier.p", "wb+" ) )
    
    sys.exit()

if __name__ == '__main__':
    UpdateClassifier()
