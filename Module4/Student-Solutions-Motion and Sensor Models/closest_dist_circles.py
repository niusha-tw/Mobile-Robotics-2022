# -*- coding: utf-8 -*-

import numpy as np
from scipy.stats import norm
import math
import matplotlib.pyplot as plt

"""
q2.1 : returns the distance to the closest intersection between a beam and an array of circles
the beam starts at (x,y) and has the angle theta (rad) to the x-axis
circles is a numpy array with the structure [circle1, circle2,...]
where each element is a numpy array [x_c, y_c, r] describing a circle 
with the center (x_c, y_c) and radius r
"""
def distance_to_closest_intersection(x, y, theta, circles):

    z_min=[]
    dis=[]
    for i in range(0,4):
      # each circle properties
      r= circles[i][2]
      x_c=circles[i][0]
      y_c=circles[i][1]
      if ( x_c < x ):
        phi= np.linspace(-1*np.pi/2,1*np.pi/2, 21)
      else :
        phi= np.linspace(1*np.pi/2,3*np.pi/2, 21)
      for j in range(0,21):
        # compute the intersection of circle points to each beam
        # the circle points in front of the robot
        x_m= r* math.cos(phi[j])+ x_c
        y_m= r* math.sin(phi[j])+ y_c
        # compute the distance between robot pose and intersection point
        dx = x_m-x
        dy = y_m-y
        dq= dx**2 +dy**2
        # compute the min distance of one beam to all the points in this loop
        q =np.sqrt(dq)
        z_min.append(q)
        # the expected distance toward the intersections
        dis_exp=np.sqrt((y_m- 0)**2 +(x_m-1)**2)
        dis.append(dis_exp)
    # compute the min distance for each beam by finding the min distance of the points that correspond to each beam 
    closest_dis = min(z_min)
    i=z_min.index(min(z_min))
    distance =dis[i]
    
    if closest_dis < 0.05 :
      return distance
    else:
      # if there is not any  intersection , show inf
      return np.inf

"""
returns the normalizer value in the hit-probability function
z_exp is the expected range (in cm)
b the variance
z_max is the maximum range (in cm) 
"""
def normalizer(z_exp, b, z_max):
    std_dev = np.sqrt(b)
    return 1.0/(norm(z_exp, std_dev).cdf(z_max) - norm(z_exp, std_dev).cdf(0.0))

def main():
    global alpha
    # define the circles in the map
    circles = np.array([[3.0, 0.0, 0.5], [4.0, 1.0, 0.8], [5.0, 0.0, 0.5], [0.7, -1.3, 0.5]])
    # robot pose
    pose = np.array([1.0, 0.0, 0.0])
    beam_directions = np.linspace(-np.pi/2, np.pi/2, 21)
    # load measurements
    z_scan = np.load('z_scan.npy')
    z_max = 10.0
    b = 1.0
    
    z_scan_exp = np.zeros(beam_directions.shape)
    for i in range(beam_directions.size):
      x_z= pose[0] + z_scan[i]*math.cos(beam_directions[i]+ pose[2])
      y_z= pose[1] + z_scan[i]*math.sin(beam_directions[i]+ pose[2])
      # compute the expected ranges using the intersection function
      z_scan_exp[i] = distance_to_closest_intersection(x_z, y_z, beam_directions[i], circles)
       
  
    ########### visualization #################################
    plt.axes().set_aspect('equal')
    plt.xlim([-0, 6])
    plt.ylim([-2, 2])
    plt.plot(pose[0], pose[1], "bo")
    fig = plt.gcf()
    axes = fig.gca()
    for i in range(beam_directions.size):
        theta = beam_directions[i]
        x_points = [pose[0], pose[0] + 10*np.cos(theta)]
        y_points = [pose[1], pose[1] + 10*np.sin(theta)]
        plt.plot(x_points, y_points, linestyle='dashed', color='red', zorder=0)

    for circle in circles:
        circle_plot = plt.Circle((circle[0], circle[1]), radius=circle[2], color='black', zorder=1)
        axes.add_patch(circle_plot)

    for i in range(beam_directions.size):
        if z_scan_exp[i] > z_max:
            continue
        theta = beam_directions[i]
        hit_x = pose[0] + np.cos(theta) * z_scan_exp[i]
        hit_y = pose[1] + np.sin(theta) * z_scan_exp[i]
        plt.plot(hit_x, hit_y, "ro")
        
        #meas_x = pose[0] + np.cos(theta) * z_scan[i]
        #meas_y = pose[1] + np.sin(theta) * z_scan[i]
        # show the measured and expected range values
        #plt.plot(meas_x, meas_y, "g*")

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.show()


if __name__ == "__main__":
    main()
