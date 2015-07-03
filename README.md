#Installation Instructions:
Download scripts in an existing ROS-Node
Download needed libraries:

	sudo apt-get install python-scipy python-sklearn

Tested on R.O.S. Hydro, Ubuntu 12.04
Python v. 2.7.3
Scipy v. 0.9.0 
Sklearn v. 0.10

laser scanner f.o.v : +- 45 deg. , intensity publishing enabled(not yet used)

# Human-Pattern-Recognition
Real time recongition of humans through laser scans

#Sample Run
	1)Record bag file with laser scans (.bag files are provided in video folder)

	2)Open a terminal and run 
		roscore

	3)On another terminal 
		cd 
	  to the directory of the scripts

	4)Convert a .bag file to .mat format by running 
		python bag2mat.py video/video2.bag video/video2.mat scan 79

	4.1)Convert as many as you want by changing the number of the video .bag file

	5)Manually annotate the data by running 
		python annotate.py 40 10 video/video2.mat

	5.1)Annotate as many as you want by changing the number of the video .mat file

	6)Create a classifier, and P.C.A. object by running 
		python merge_train.py video/

	7)Test online with the previously created classifier by running 
		python hpr_with_metric.py video/Gaussian_NB_classifier_merged.p video/PCA_object.p scan 40 10

	8)In another terminal 
		cd 
	  to the directory of the scripts.

	9)Run 
		rosbag play video/video10.bag 
	  so that the script in the previous step is triggered.
	
#Python files explained
#----------------------

#a)bag2mat.py:

	Convert a .bag file to .mat format.
	

Command line use:

	$python bag2mat.py <.bag_file_path> <.mat_file_path> <laser_scan_rostopic> <scan_duration>

#b)annotate.py :

Annotate (label) each cluster as human or not human, in order to train the classifier later.
annotate.py generates many .p files, some of which are classifiers trained on each and only file 
annotated. This means that every time the annotation process ends, a new classifier is created trained
only on the .mat that was just annotated.
	

Command line use:

	$python annotate.py <time_window> <number_of_frames_to_create_walls> <.mat_file_path>

#c)merge_train.py:

Merge_train uses all the annotations from a folder, to create a classifier based on all of those
annotations.


Command line use:

	$python merge_train <folder of annotated .mat files>
	
#d)hpr.py:

Runs the human pattern recognition (Naive Bayes) classifier.


	Command line use:

	$python hpr.py <classifier_object_path> <pca_objec_path> <laserscan_topic> <timewindow_in_frames> <maximum_scan_range>

#e)hpr_with_metrics.py
	
Runs the human pattern recognition (Naive Bayes) classifier and generates a classification_results.mat file that contains its results.
	

	Command line use :
	
	$rosrun hpr_with_metrics.py <classifier_object_path> <pca_objec_path> <laserscan_topic> <timewindow_in_frames> <maximum_scan_range>

#f)annotate_for_metrics.py
	
Runs the mat file that the classifier was tested on, and lets the user annotate the same clusters as the classifier in order to generate
some basic metrics (Precision, Recall, Accuracy).


	Command line use :
	
	$rosrun annotate_for_metrics.py </path/to/classification_results.mat>

#g)offline_test.py :
	todo
	$python offline_test.py <data_file_path> <annotation_data> <classifier_path> <pca_object_path> <timewindow> <frames for walls>

#h)Library files
	todo

RECOMMENDATION:

	Use same timewindow, and wall set time for each training set, and use the same values when
	evaluating
    
