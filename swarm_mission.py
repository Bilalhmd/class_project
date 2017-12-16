# this the first swarm mission attempt

import swarm_bots as sb
import numpy as np
import time

def initialize_swarm():
    """ creates a list with all robot objects defined through swarm_bots class"""
    bots = []
    for i in range(6):
        bots += [sb.Robot(id=i, dt=.001, debug=False)]
        bots[i].interf.set_wheel_velocity([0.,0.])
    return bots


def mission(robots, target_position=[0., 0.], formation=None,
            gains=[[1., 1.], [1.5, 1.5], [1., 1.]]):
    status = []
    for bot in range(len(robots)):
        status += [True]
    while any(status):
        for bot in range(len(robots)):
            status[bot] = robots[bot].controller(target_position, formation, gains)
    print 'mission completed'


















if __name__ == "__main__":
    sb.reset_world()
    time.sleep(.5)
    robots = initialize_swarm()
    sb.reset_world()
    time.sleep(.5)
    mission(robots, target_position=[2., -1.], formation='Escape',
            gains=[[.75,.75],[1.5,1.5],[1., 1.]])
    time.sleep(2.)
    mission(robots, target_position=[2., -11.], formation='Escape',
            gains=[[1.,1.],[1.5,1.5],[1., 1.]])
    # time.sleep(2.)
    # mission(robots, target_position=[2., -1.], formation='Line')