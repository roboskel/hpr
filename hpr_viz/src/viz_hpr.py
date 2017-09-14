#!/usr/bin/env python
import roslib, rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#matplotlib.rcParams['backend'] = "Qt4Agg"
from laser_wall_extraction.msg import WallVizMsg
from laser_clustering.msg import ClustersMsg

from mayavi import mlab

import curses 

num_of_diagrams = 2
pause_function = False
stdscr = None
pause = False
plot_figure = None
wall_x = []
wall_y = []
top_view_figure = None
cc  =  ['r', 'c', 'k',  '#FF6600', '#990099', '#0000FF', 'g','#8B4513','y','#FFD700']

colors = [(1, 0, 0), (0, 1, 1), (0, 0, 1), (0, 1, 0), (0.5, 0.5, 0.5), (1, 0.0784314, 0.576471), (0.133333, 0.545098, 0.133333), (1, 0.843137, 0), (0.580392, 0, 0.827451), (0.823529, 0.411765, 0.117647)]
mm = ['2dcircle', '2dcross', '2ddiamond', '2dsquare', '2dthick_cross', '2dtriangle', '2dvertex', 'cube']

clear_top_plot = True
clear_clusters_plot = True
clear_overlap_plot = True

img_counter = 1
path = '/home/turtlebot/hpr_images/'
wall_sub = None


def init():
    global num_of_diagrams, pause_function
    global plot_figure, top_view_figure, clusters_plot, overlap_plot
    global trace_3d_figure, stdscr, wall_sub

    rospy.init_node('hpr_viz')

    wall_input_topic = rospy.get_param('~wall_input_topic', 'laser_wall_extraction/viz_req')
    clustering_input_topic = rospy.get_param('~clustering_input_topic', 'laser_clustering/clusters')
    overlap_input_topic = rospy.get_param('~overlap_input_topic', 'laser_overlap_trace/tracks')
    num_of_diagrams = rospy.get_param('~num_of_diagrams', 3)
    pause_function = rospy.get_param('~pause_function', False)

    if pause_function:
        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(1)
        stdscr.timeout(0);

    wall_sub = rospy.Subscriber(wall_input_topic, WallVizMsg, plot_walls)

    rospy.Subscriber(clustering_input_topic, ClustersMsg, plot_clustering)

    rospy.Subscriber(overlap_input_topic, ClustersMsg, plot_tracks)

    
    if num_of_diagrams > 2:
        rospy.Subscriber(overlap_input_topic, ClustersMsg, plot_overlap)

    if num_of_diagrams == 2:
        plot_figure = plt.figure() 
        top_view_figure = plot_figure.add_subplot(1, 2, 1)
        top_view_figure.set_title("Top view")
        top_view_figure.set_xlabel('Vertical distance')
        top_view_figure.set_ylabel('Robot is here')

        clusters_plot = plot_figure.add_subplot(1, 2, 2, projection='3d')
        clusters_plot.set_title("3D view")
        clusters_plot.set_xlabel('X - Distance')
        clusters_plot.set_ylabel('Y - Robot')
        clusters_plot.set_zlabel('Z - Time')

    elif num_of_diagrams == 3:
        plot_figure = plt.figure()
        top_view_figure = plot_figure.add_subplot(2, 2, 1)
        top_view_figure.set_title("Top view")
        top_view_figure.set_xlabel('Vertical distance')
        top_view_figure.set_ylabel('Robot is here')

        clusters_plot = plot_figure.add_subplot(2, 2, 2, projection='3d')
        clusters_plot.set_title("3D view")
        clusters_plot.set_xlabel('X - Distance')
        clusters_plot.set_ylabel('Y - Robot')
        clusters_plot.set_zlabel('Z - Time')

        overlap_plot = plot_figure.add_subplot(2, 2, 3, projection='3d')
        overlap_plot.set_title("Tracked 3D clusters")
        overlap_plot.set_xlabel('X - Distance')
        overlap_plot.set_ylabel('Y - Robot')
        overlap_plot.set_zlabel('Z - Time')

    elif num_of_diagrams == 1:
        plot_figure = plt.figure()
        top_view_figure = plot_figure.add_subplot(1, 1, 1)
        top_view_figure.set_title("Top view")
        top_view_figure.set_xlabel('Vertical distance')
        top_view_figure.set_ylabel('Robot is here')

    plt.show()

    
    while not rospy.is_shutdown():  
        rospy.spin()
    if pause_function:
        curses.nocbreak()
        stdscr.keypad(0)
        curses.echo()
        curses.endwin()

def plot_walls(wall_data):
    global wall_x, wall_y, top_view_figure, wall_sub
    wall_x = wall_data.x
    wall_y = wall_data.y
    top_view_figure.clear()
    top_view_figure.set_title("Top view")
    top_view_figure.set_xlabel('Vertical distance')
    top_view_figure.set_ylabel('Robot is here')
    top_view_figure.plot(wall_x, wall_y)
    plt.draw()
    wall_sub.unregister()

def plot_clustering(data):
    global num_of_diagrams, top_view_figure, clusters_plot, wall_x, wall_y, pause, stdscr, clear_top_plot, clear_clusters_plot
    global img_counter, path


    if(len(data.array_sizes) > 0):
        if pause_function:
            key = stdscr.getch()
            stdscr.refresh()
            if key == curses.KEY_ENTER or key == 10: #ENTER TO PAUSE THE DIAGRAMS ;)
                pause = not pause
                print 'Pause = \033[93m '+ str(pause) + ' \033[0m'

        if not pause:
            clear_top_plot = False
            top_view_figure.clear()
            top_view_figure.set_title("Top view")
            top_view_figure.set_xlabel('Vertical distance')
            top_view_figure.set_ylabel('Robot is here')
            top_view_figure.plot(wall_x, wall_y)
            top_view_figure.scatter(data.x, data.y, 20, 'red')

            if num_of_diagrams > 1:
                clear_clusters_plot = False
                clusters_plot.clear()
                clusters_plot.set_title("3D view")
                clusters_plot.set_xlabel('X - Distance')
                clusters_plot.set_ylabel('Y - Robot')
                clusters_plot.set_zlabel('Z - Time')

                prev_index = 0
                for i in range(0, len(data.array_sizes)):
                    xk = []
                    yk = []
                    zk = []

                    for j in range(prev_index, prev_index+data.array_sizes[i]-1):
                        xk.append(data.x[j])
                        yk.append(data.y[j])
                        zk.append(data.z[j])
                    prev_index = data.array_sizes[i] 

                    j=data.id_array[i]
                    clusters_plot.scatter(xk, yk, zk, 'z', 30, cc[j%10])


            plt.draw()
            #plt.savefig(path+'clusters_'+str(img_counter), format='png')
    
    else:
        if not clear_top_plot:
            top_view_figure.clear()
            top_view_figure.set_title("Top view")
            top_view_figure.set_xlabel('Vertical distance')
            top_view_figure.set_ylabel('Robot is here')
            top_view_figure.plot(wall_x, wall_y)
            clear_top_plot = True
            plt.draw()
        if num_of_diagrams > 1 and not clear_clusters_plot:
            clusters_plot.clear()
            top_view_figure.set_title("Top view")
            top_view_figure.set_xlabel('Vertical distance')
            top_view_figure.set_ylabel('Robot is here')
            clear_clusters_plot = True
            plt.draw()
    

def plot_overlap(data):
    global overlap_plot, pause, clear_overlap_plot
    global img_counter, path

    array_sizes = np.array(data.array_sizes)
    idArray = np.array(data.id_array)
    prev_index = 0


    if(len(data.array_sizes) > 0):
        if not pause:

            overlap_plot.clear()
            overlap_plot.set_title("Tracked 3D clusters")
            overlap_plot.set_xlabel('X - Distance')
            overlap_plot.set_ylabel('Y - Robot')
            overlap_plot.set_zlabel('Z - Time')
            clear_overlap_plot = False

            for i in range(0, len(array_sizes)):
                xk = []
                yk = []
                zk = []

                for j in range(prev_index, prev_index+array_sizes[i]-1):
                    xk.append(data.x[j])
                    yk.append(data.y[j])
                    zk.append(data.z[j])
                prev_index = array_sizes[i] - 1

                x_, y_, z_ = median_viz(xk,yk,zk)

                overlap_plot.scatter(x_, y_, z_, 'z', 30, cc[idArray[i]%10])

       
            plt.draw()
            #plt.savefig(path+'tracedCl_'+str(img_counter), format='png')
            
            img_counter += 1
    else:
        if not clear_overlap_plot:
            overlap_plot.clear()
            overlap_plot.set_title("Traced 3D clusters")
            overlap_plot.set_xlabel('X - Distance')
            overlap_plot.set_ylabel('Y - Robot')
            overlap_plot.set_zlabel('Z - Time')
            clear_overlap_plot = True
            plt.draw()

def median_viz(x, y, z):
    split = 16
    X_ = []
    Y_ = []
    Z_ = []

    split_count = 0

    while split_count < len(x):
        xmed = np.median(x[split_count:split_count+split])
        ymed = np.median(y[split_count:split_count+split])
        zmed = np.median(z[split_count:split_count+split])

        if ((math.isnan(xmed)) or (math.isnan(ymed)) or (math.isnan(zmed))):
            split_count += split
            continue

        X_.append(xmed)
        Y_.append(ymed)
        Z_.append(zmed)
        split_count += split

    return X_,Y_,Z_


def plot_tracks(data):
    global overlap_plot, pause, clear_overlap_plot
    global img_counter, path

    array_sizes = np.array(data.array_sizes)
    idArray = np.array(data.id_array)
    prev_index = 0

    if(len(data.array_sizes) > 0):
        if not pause:
            myfig = mlab.figure(1, fgcolor=(0, 0, 0), bgcolor=(1, 1, 1), size=(900,900))
            mlab.clf(myfig)

            for i in range(0, len(array_sizes)):
                xk = []
                yk = []
                zk = []

                for j in range(prev_index, prev_index+array_sizes[i]-1):
                    xk.append(data.x[j])
                    yk.append(data.y[j])
                    zk.append(data.z[j])
                prev_index = array_sizes[i] - 1

                ind = idArray[i]    #%len(colors)
                #xk, yk, zk = median_viz(xk,yk,zk)

                pts = mlab.points3d(xk, yk, zk, mode=mm[ind%len(mm)], color=colors[ind%len(colors)], scale_factor = 0.15)

                #mlab.axes(figure=myfig)
            #mlab.xlabel("X - Distance")
            #mlab.ylabel("Y - Robot")
            #mlab.zlabel("Z - Time")
            mlab.show()
    
    else:
        if not clear_overlap_plot:
            mlab.clf(myfig)
            clear_overlap_plot = True

            mlab.show()
    


if __name__ == '__main__':
    init()   
