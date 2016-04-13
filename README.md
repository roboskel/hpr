#Installation Instructions:
Download this ROS package inside a catkin workspace.

#Working with the below library versions:

scikit-image==0.12.3
scikit-learn==0.15.1
scipy==0.13.3
numpy==1.10.1
matplotlib==1.3.1

It is really important to install the indicated versions of the libraries because older versions
are much slower. (pip recommended)

RECOMMENDATIONS:
	
	Use same timewindow, and wall set time for each training set, and use the same values when
	evaluating.

#Running out-of-the-box
	roslaunch human_pattern_recognition hpr.launch
	The script is now waiting for laser scans to be published in the topic /scan. If you want to change the parameters of the script, just edit hpr.launch, or create your own .launch file.

# Human-Pattern-Recognition
Real time recongition of humans through laser scans
	
#Python files explained
#----------------------

#I)bag2mat.py:

	Convert a .bag file to .mat format.
	

Command line use:

	python bag2mat.py <.bag_file_path> <.mat_file_path> <laser_scan_rostopic> <scan_duration>

#II)annotate.py :

Annotate (label) each cluster as human or not human, in order to train the classifier later.
annotate.py generates many .p files, some of which are classifiers trained on each and only file 
annotated. This means that every time the annotation process ends, a new classifier is created trained
only on the .mat that was just annotated.
	

Command line use:

	python annotate.py <time_window> <number_of_frames_to_create_walls> <.mat_file_path>

#III)merge_train.py:

This is an old file. Use recognition_procedure instead. Merge_train uses all the annotations from a folder, to create a classifier based on all of those
annotations.


Command line use:

	python merge_train <folder of annotated .mat files>
	
#IV)hpr.py:

Runs the human pattern recognition classifier.


Command line use:

	roslaunch human_pattern_recognition hpr.launch

#V)generate_metrics.py
	
Runs the mat file that the classifier was tested on, and lets the user annotate the same clusters as the classifier in order to generate
some basic metrics (Precision, Recall, Accuracy).


Command line use :
	
	python generate_metrics.py </path/to/classification_results.mat>

#VI)recognition_procedure.py
The python script that is now used to train the classifier. Three different classifiers can be trained, but LDA seems to be the best.

#VII)Library files
gridfit.py, myhog.py and mytools.py are libraries used to run the code. They were pre-written by others or ported to python.


    
