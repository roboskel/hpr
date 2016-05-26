#!/usr/bin/env python
import roslib, rospy
import numpy as np
import matplotlib.pyplot as plt

num_of_diagrams = 2
pause_function = False
stdscr = None
pause = False
plot_figure = None

plt.ion()


def init():
    global num_of_diagrams, pause_function
    global plot_figure, top_view_figure, ax, ax3, num_of_diagrams
    global trace_3d_figure

    rospy.init_node('hpr_viz')

    input_topic = rospy.get_param('~input_topic','laser_analysis/viz_req')
    num_of_diagrams = rospy.get_param('~num_of_diagrams',2)
    pause_function = rospy.get_param('~num_of_diagrams',False)
   
    rospy.Subscriber(input_topic, BufferMsg, clustering_procedure)

    clusters_publisher = rospy.Publisher(clusters_topic, ClustersMsg, queue_size=10)

    if num_of_diagrams == 2:
        plot_figure = plt.figure() 
        top_view_figure = plot_figure.add_subplot(1,2,1)
        top_view_figure.set_title("Top view")
        top_view_figure.set_xlabel('Vertical distance')
        top_view_figure.set_ylabel('Robot is here')
        #top_view_figure.plot(wall_cart[:,0],wall_cart[:,1])

        ax = plot_figure.add_subplot(1,2,2,projection='3d')
        ax.set_title("3D view")
        ax.set_xlabel('X - Distance')
        ax.set_ylabel('Y - Robot')
        ax.set_zlabel('Z - Time')

    elif num_of_diagrams == 3:
        plot_figure = plt.figure()
        top_view_figure = plot_figure.add_subplot(2,2,1)
        top_view_figure.set_title("Top view")
        top_view_figure.set_xlabel('Vertical distance')
        top_view_figure.set_ylabel('Robot is here')
        #top_view_figure.plot(wall_cart[:,0],wall_cart[:,1])

        ax = plot_figure.add_subplot(2,2,2,projection='3d')
        ax.set_title("3D view")
        ax.set_xlabel('X - Distance')
        ax.set_ylabel('Y - Robot')
        ax.set_zlabel('Z - Time')

        ax3 = plot_figure.add_subplot(2,2,3,projection='3d')
        ax3.set_title("Traced 3D clusters")
        ax3.set_xlabel('X - Distance')
        ax3.set_ylabel('Y - Robot')
        ax3.set_zlabel('Z - Time')

    elif num_of_diagrams == 1:
        plot_figure = plt.figure()
        top_view_figure = plot_figure.add_subplot(1,1,1)
        top_view_figure.set_title("Top view")
        top_view_figure.set_xlabel('Vertical distance')
        top_view_figure.set_ylabel('Robot is here')
        #top_view_figure.plot(wall_cart[:,0],wall_cart[:,1])

 

    plt.show()
    trace_3d_figure.show()

    while not rospy.is_shutdown():  
        rospy.spin()

def update_plots(data):
    

if __name__ == '__main__':
    init()   