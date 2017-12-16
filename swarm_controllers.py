import gazebo_swarm
import time
import numpy as np

def controller(robot_num, interface):
    ##get sensors
    
    ## this returns the left wheel position,
    # right wheel position, left wheel velocity
    # and right wheel velocity
    wheels = interface.get_wheel_state()
    
    ##this returns a list of neighbors within
    # a 2 meter range
    ##each element contains the neighbor robot
    # number and a 3d vector for the relative
    # position of the agent
    neighbors = interface.get_neighbors()
    
    ##this returns a list containing the
    # distance sensor values
    ##there are 3 distance sensors
    # (front_dist_sensor, left_dist_sensor,
    # right_dist_sensor)
    ##which contain 20, 10, 10 rays each
    # (can sense obstacles up to 1.5meters)
    dist_sense = interface.get_distance_sensors()
    
    #here we get the GPS if the robot is number 0
    if robot_num==0:
        gps_position = interface.get_gps()
    
    ##do something - here we implement a silly
    # consensus protocol to show how things work
    desired_vel = [0.,0.]
    des_vel = np.array([0.,0.])
    for n in neighbors:
        des_vel[0] += 0.3*n[1][0] *10
        des_vel[1] += 0.3*n[1][1] *10
    desired_vel[0] = des_vel[0] - des_vel[1]
    desired_vel[1] = des_vel[0] + des_vel[1]
    
    ##send desired wheel velocity (2 numbers -
    # left and right wheel velocity)
    interface.set_wheel_velocity(desired_vel)
    
def run():
    #get an interface to each robot
    interf = []
    for i in range(6):
        interf.append(gazebo_swarm.GazeboRobotIface(i))

    while(True):
        for i in range(6):
            controller(i,interf[i])
        ##pause for 100ms
        time.sleep(0.1)
        
def reset_world():
    inter = gazebo_swarm.GazeboWorldIface()
    inter.reset()