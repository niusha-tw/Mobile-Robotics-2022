# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import *
import numpy as np

MAX_SPEED =12.23
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
# create the Robot instance.
v= 12.23
w=0.0
R=0.195/2.0
L=0.325
yg=4.0
xg=0.0
phi=0.0
refd=1.57
x=0.0
y=0.0
v_r = (2*v + (w*L)) / 2*R
v_l = (2*v - (w*L)) / 2*R
    # set wheel velocities (in rad)
    # a velocity of 2*pi means that the wheel will make one turn per second
leftWheel.setVelocity(v_l)
rightWheel.setVelocity(v_r)
    # get and enable lidar
# Main loop:
    # - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
       
        dt=0.1  
        x =x + v*cos(phi)
        y = y + v*sin(phi)
        phi = phi + w
        # Position error:
        x_err = xg - x
        y_err = yg - y
        dist_err = sqrt(pow((x_err), 2) + pow((y_err), 2))
        # Orientation error
        phi_d = np.arctan2(y_err,x_err)
        phi_err = phi_d - phi
        # Limit the error to (-pi, pi):
        phi_err_correct = np.arctan2(np.sin(phi_err),np.cos(phi_err))
        #print(phi_err_correct,dist_err,phi_d)                 # Proportional term; kp is the proportional gain
        # controller output
       
        if dist_err >= 0.1:
            w=20*phi_err
            v_r = (2*v + (w*L)) / 2*R
            v_l = (2*v - (w*L)) / 2*R
                # set wheel velocities (in rad)
                # a velocity of 2*pi means that the wheel will make one turn per second
            leftWheel.setVelocity(v_l)
            rightWheel.setVelocity(v_r)
        else:
            leftWheel.setVelocity(0.0)
            rightWheel.setVelocity(0.0)
       
        pass