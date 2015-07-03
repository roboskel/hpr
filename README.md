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
	2)cd to the directory of the scripts
	3)Convert the desired .bag file to .mat format by running python bag2mat.py video/video2.bag video/video2.mat scan 79
	4)Manually annotate the data by running python annotate.py 40 10 video/video2.mat
	5)Create a classifier, and P.C.A. object by running python merge_train.py video/
	6)Test online with the previously created classifier by running python hpr_with_metric.py video/Gaussian_NB_classifier_merged.p video/PCA_object.p scan 40 10
	7)In another terminal, cd to the directory of the scripts.
	8)Run rosbag play video/video10.bag so that the script in step 6 is triggered.
	
#a)Convert R.O.S. bagfiles to suitable .mat files using 'bag2mat.py':

	Enter desired destination with file ending in .mat
	
Command line use:

	$rosrun <package_name> bag2mat.py <bag_file_path> <.mat_file_path> <laser_scan_rostopic> <scan_duration>

#b)Annotate with annotate.py :

Either provide command line arguments with the same order as below, or run the script without arguments and provide them when prompted

	*Enter timewindow (int)
	
	*Enter frames to set wall (int)
	
	*Enter filename (string, no quotes)
	
	trained data will be saved as : <input>.<trainingdata>
	

Command line use:

	$python annotate.py <time_window> <wall_set_frames> <mat_file_to_use>

#c)create classifier with merge_train.py:

merge_train will create a classifier in the specified folder

	$python merge_train <folder of annotated .mat files>
	
#d)Test on live data with hpr_with_metrics.py:

	Publish laser scans on topic /scan, enable intensities, set min_angle, max_angle to -45,45 degrees
	respectively (to be changed).
	
Command line use :
	$rosrun <package_name> hpr.py <classifier object path> <pca objec path> <laserscan topic> <timewindow in frames> <maximum scan range>

#e)Test with data files instead of live data :
	$python offline_test.py <data_file_path> <annotation_data> <classifier_path> <pca_object_path> <timewindow> <frames for walls>

RECOMMENDATION:

	Use same timewindow, and wall set time for each training set, and use the same values when
	evaluating
    
