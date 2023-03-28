from controller import Robot
from math import *
import numpy as np

# Maximum speed for the velocity value of the wheels.
# Don't change this value.
MAXSPEED = 5.24
# Get pointer to the robot.
robot = Robot()
# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# Get pointer to the robot wheels motors.
leftWheel = robot.getDevice('left wheel')
rightWheel = robot.getDevice('right wheel')
# We will use the velocity parameter of the wheels, so we need to
# set the target position to infinity.
leftWheel.setPosition(float('inf'))
rightWheel.setPosition(float('inf'))
# Get and enable the distance sensors.
lidar = robot.getDevice('Sick LMS 291')
lidar.enable(100)
lidar.enablePointCloud()
# Main loop:
# Move forward until we are 50 cm away from the wall.    
# Don't change this value.
w=0.0
R=0.195/2.0
L=0.325
phi=0.0
x=-1.0
y=2.0
ref=1.57
refd=3.1457
v= 0.0
w=30
v_r = (2*v + w*L) / 2*R
v_l = (2*v - w*L) / 2*R
leftWheel.setVelocity(v_l)
rightWheel.setVelocity(v_r)
   
while robot.step(timestep) != -1:
    scan = lidar.getRangeImage()
    # Rotate clockwise until the wall is to our left.
    # Main loop.
    s=min(scan)
    teta=scan.index(s)
    teta=teta*(3.14/180.0)
    dt=0.1
    xmin=s*cos(teta+phi)+x-0.08*cos(phi)
    ymin=s*sin(teta+phi)+y-0.08*sin(phi)
    x += v*cos(phi)*dt
    y += v*sin(phi)*dt
    phi += w*dt
     # Position error:
    x_err = x - xmin
    y_err = y - ymin
    dist_err = np.sqrt(x_err**2 + y_err**2)
    phi_d = np.arctan2(y_err,x_err)
    phi_err = ref - teta
    print(dist_err)
    # Limit the error to (-pi, pi):
    phi_err_correct = np.arctan2(np.sin(phi_err),np.cos(phi_err))   
    # Move forward until we are 50 cm away from the wall.        
    if s>0.50:
        v= 8.0
        w=30*phi_err_correct
        v_r = (2*v + w*L) / 2*R
        v_l = (2*v - w*L) / 2*R
        leftWheel.setVelocity(v_l)
        rightWheel.setVelocity(v_r)      
    else:    
        v=0.0
        w=0.0
        v_r = (2*v + w*L) / 2*R
        v_l = (2*v - w*L) / 2*R
        leftWheel.setVelocity(v_l)
        rightWheel.setVelocity(v_r)    
