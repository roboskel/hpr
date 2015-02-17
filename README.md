# Human-Pattern-Recognition
Real time recongition of humans through laser scans

a)Convert R.O.S. bagfiles to suitable .mat files using 'bag2mat.py':

	Enter desired destination with file ending in .mat

b)Train with offline_train.py:

	*Enter timewindow (int)
	
	*Enter frames to set wall (int)
	
	*Enter max range scanned (float)

	trained data will be saved as : <input>.<trainingdata>
	!!!When asked for max laser range, input the maximum range scanned 
	during the recording, not the maximum range at which you want to train

c)Test on live data with hpr.py:
	Publish laser scans on topic /scan, enable intensities, set min_angle, max_angle to -45,45 degrees
	respectively (to be changed).
	

RECOMMENDATION: Use same timewindow, and wall set time for each training set, and use the same values when
	evaluating
    
