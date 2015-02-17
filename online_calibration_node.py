#!/usr/bin/env python
__author__="athanasia sapountzi"

import roslib,rospy,pickle

import numpy as np
import scipy.io as sio
from matplotlib.pyplot import *

from sensor_msgs.msg import LaserScan
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.interpolate import griddata,lagrange,interp1d
counter=1

ion()

def laser_listener():
    
    global publisher,rmax,sampling,limit

    print 'Initialize node ..'
    rospy.init_node('laser_listener', anonymous=True)
    sub_topic = "scan"
    print 'insert max polar distance:'
    rmax=input()
   
    print 'Insert time limit for processing:'
    limit=40*input()
    print 'Subscribing to :', sub_topic
    rospy.Subscriber(sub_topic,LaserScan,collect_data)
    rospy.spin()

def collect_data(laserscan):

    global phi, A, counter,sampling,limit,rmax,limit
    
    if counter==1:
        print 'Reduce points by 2? 1/0'
        if input()==1 :
            sampling=np.arange(0,len(np.array(laserscan.ranges)),2)#apply sampling e.g every 2 steps
        else :
            sampling=np.arange(0,len(np.array(laserscan.ranges)),1)

        #collect sparse data
        phi=np.arange(laserscan.angle_min,laserscan.angle_max,laserscan.angle_increment)[sampling]
        rng=np.array(laserscan.ranges)[sampling]
        intens=np.array(laserscan.intensities)[sampling]

        #reduce max range
        filter=np.where(rng<rmax)
        rng=rng[filter]
        theta=phi[filter]
        intens=intens[filter]
        ranges=rng

        print 'Min_angle :', round(np.rad2deg(theta[0])),'degrees'
        print 'Max angle :', round(np.rad2deg(theta[len(theta)-1])),'degrees'
        print 'Initial sampling Step: ', np.rad2deg(laserscan.angle_increment),'degrees'

        A=pol2cart(rng,theta,intens)  #A=[x,y,intensity,range,theta]
        [x,y,intensities,ranges,th]=[A[:,0],A[:,1],A[:,2],A[:,3],A[:,4]]

        katopsi(x,y)
        print 'Assembling data ...'

    else:
        #A=[x,y,intensity,range,theta,counter]

        intens=np.array(laserscan.intensities)[sampling]
        rng=np.array(laserscan.ranges)[sampling]
        filter=np.where(rng<rmax)
        rng=rng[filter]
        theta=phi[filter]
        intens=intens[filter]
        
        temp=pol2cart(rng,theta,intens)

        A=np.vstack((A,temp))   #A=[x,y,intensity,range,theta]

    if counter == limit :

        [x,y,intensities,ranges,th]=[A[:,0],A[:,1],A[:,2],A[:,3],A[:,4]]

        multi_plots(x,y,intensities,ranges,th)
        #surf(x,y,intensities)
        save(np.array([x,y]),intensities,ranges,th)
                
    counter=counter+1

def pol2cart(r,theta,zed):
    x=np.multiply(r,np.cos(theta))
    y=np.multiply(r,np.sin(theta))
    C=np.array([x,y,zed,r,theta]).T

    return C

def scatter_polar(r,theta,rmax):
    figurename=plt.figure()
    ax = figurename.subplot(111, polar=True)
    ax.scatter(theta, r, color='r')
    ax.set_rmax(rmax)
    ax.grid(True)
    ax.set_title("data in polar c.s.", va='bottom')
    figurename.show()
    figurename.savefig('scatter_polar.png')

def scatter_xyz(x,y,z,labx,laby,labz):
    figurename=figure()
    ax = figurename.gca(projection='3d')
    ax.scatter(x, y, z, 'z', 20)
    ax.set_xlabel(labx)
    ax.set_ylabel(laby)
    ax.set_zlabel(labz)
    figurename.show()
    figurename.savefig('scatter_xyz')
    
def save(cart,intensities,ranges,phi):

    print 'saving data ...'
    pickle.dump(intensities, open("intensities.p","wb"))
    pickle.dump(ranges, open("ranges.p","wb"))
    pickle.dump(cart, open("cartesians.p","wb"))
    pickle.dump(phi, open("theta.p","wb"))

    b={}
    b['intensities']=intensities    #slot number of each point
    b['ranges']=ranges              #cartesian coordinates of cluster points
    b['rad_angles']=phi
    b['cartesians']=cart
    sio.savemat('apostasiometrika',b)

    print 'data saved'

def interpolation(x,y):

    fig = figure()
    sub = kat.add_subplot(111)
    f=interp1d(x,y)
    sub.plot(x, y, 'o')# xnew, ynew, '-')
    sub.show()

def surf(x,y,z):

    fig = figure()
   
    ax = fig.gca(projection='3d')
    yi,zi=np.meshgrid(y,z, sparse=True)
    xi = griddata(y,z,x,yi,zi)
    surf = ax.plot_surface(xi, yi, zi, rstride=6, cstride=6, cmap=cm.jet,linewidth=0)
    ax.contourf(x,y,z)
    ax.set_zlim3d(min(z), max(z))

    ax.w_zaxis.set_major_locator(LinearLocator(10))
    ax.w_zaxis.set_major_formatter(FormatStrFormatter('%.03f'))
    fig.colorbar(surf, shrink=0.5, aspect=5)

    fig.show()

def katopsi(x,y):
    kat = figure()
    ax = kat.add_subplot(111)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.scatter(x,y)
    ax.set_title('katopsi 1o scan', va='bottom')
    kat.show()
    kat.savefig('katopsi.png')

def multi_plots(x,y,z,r,th):

    th=np.rad2deg(th)
    thmax=max(th)
    thmin=min(th)
    band=max(th)-min(th)
    print len(r),'points'
    print 'bandwidth=',band,'deg'
    print 'insert min slot size in degrees'
    band=input()
    
    la=np.arange(min(th),max(th),band)
    la[len(la)-1]=max(th)
    
    nof=len(la) #number of figures

    j=(nof/2)
  
    fig = figure()
    ax = fig.add_subplot(1,1,1)
    ax.scatter(r,z)
    ax.grid(True)
    ax.set_title((round(min(th)),'<a<',round(max(th))), va='bottom')
    fig.show()
    fig.savefig('all_points.png')

    fig2=figure()
    for i in range(0,nof-1):
        h=np.where(th<la[i+1])
        th1=th[h]
        r1=r[h]
        z1=z[h]
        l=np.where(th1>la[i])
        r1=r1[l]
        z1=z1[l]
        ax1 = fig2.add_subplot(2,j,i+1, sharex = ax, sharey = ax)
        ax1.scatter(r1, z1)
        ax1.grid(True)
        ax1.set_title((round(la[i]),'<a<',round(la[i+1])), va='bottom')
        
    fig2.show()
    fig2.savefig('multiplots.png')
    print 'pause trick'
    
    input() #pause plotting
    
if __name__ == '__main__':
    laser_listener()
