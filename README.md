# Human-Pattern-Recognition
Real time recongition of humans through laser scans

a)Convert R.O.S. bagfiles to suitable .mat files using 'bag2mat.py'
	Create a folder named 'mat_files' (to be corrected)
	Enter desired destination file name as '<x>.mat', including the single quotes
	file will be saved as : ./mat_files/<input>

b)Train with offline_train.py:
	When asked, input file should be : 'mat_files/<mat_file>', 
	including the single quotes

	trained data will be saved in as : mat_files/<data_files>
	!!!When asked for max laser range, input the maximum range scanned 
	during the recording, not the maximum range at which you want to train

c)Test on live data with hpr.py:
	Publish laser scans on topic /scan, enable intensities, set min_angle, max_angle to -45,45 degrees
	respectively (to be changed).
	

RECOMMENDATION: Use same timewindow, and wall set time for each training set, and use the same values when
	evaluating
    
