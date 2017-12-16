import gazebo_swarm
import swarm_formation as sf
import time
import numpy as np



class Robot(object):
    def __init__(self, id, dt, debug=False):
        self.interf = gazebo_swarm.GazeboRobotIface(id)
        self.id = id
        self.dt = dt
        self.r = 0.033     # wheel radius taken from gazebo world
        self.l = 0.26   # distance separating both wheels taken from gazebo world
        self.param = self.r/self.l
        # self.heading = 0.  # initial robot heading
        self.current_heading = 0.
        self.debug = debug
        self.sensor_angles = [np.pi/3, -np.pi/3, 0.] # L, R, M
        self.formation = sf.SwarmFormation(self.interf,self.id)
        self.kp_omega, self.kd_omega = 2.5, .3
        self.kp_vel, self.kd_vel = 1.5, 1.

    def position(self):
        if self.id == 0:
            return self.interf.get_gps()
        else:
            return [0., 0., 0.]

    def wheel_states(self):
        return self.interf.get_wheel_state()

    def wheel_speeds(self, v, w):
        """ returns wheel velocity [vl, vr] from given velocities """
        return [(2*v - w * self.l)/(2*self.r),
                (2*v + w * self.l)/(2*self.r)]

    def angular_velocity(self):
        states = self.wheel_states()
        return self.param*(states[3]-states[2])

    def local_to_global(self, dx, dy):
        """ transforms a vector [dx, dy] from robot local frame to
            the global frame.
        """
        return np.cos(self.current_heading)*dx + \
               -np.sin(self.current_heading)*dy, \
               np.sin(self.current_heading)*dx + \
               np.cos(self.current_heading)*dy

    def integrate_heading(self):
        current_rotation = self.wheel_states()
        self.current_heading = current_rotation[1]-current_rotation[0]
        self.current_heading *= self.param
        if self.debug : print 'current_angle', self.current_heading

    def distance_sensors(self):
        # reads all sensors and returns all readings
        readings = self.interf.get_distance_sensors()
        return readings[0][1], readings[1][1], readings[2][1]

    def obstacle_detection(self):
        measurements = self.distance_sensors()
        obstacles = []
        for sensor in measurements:
            total_distance = 0.
            k = 0
            for reading in sensor:
                if not np.isinf(reading):
                    total_distance += reading
                    k += 1
            if k == 0:
                average = None
            else:
                average = total_distance/k
            obstacles += [average]
        return obstacles

    def obstacle_gaussian(self, obstacle_distance, sensor_angle=0.,
                          gain=3.7, variance=.12):
        """ returns 2D gaussian function for the obstacle """
        if obstacle_distance is None:
            # if no obstacle for current sensor reading
            return 0., 0.
        else:
            f0 = (gain / np.sqrt(2 * np.pi * (variance ** 2))) * np.exp(
                (- 0.5 * obstacle_distance ** 2) / (variance ** 2))
            fx = - f0 * np.cos(sensor_angle)
            fy = - f0 * np.sin(sensor_angle)
            return fx, fy

    def obstacle_control(self, gains=[1.,1.]):
        ## first read sensors and return obstacle locations
        ## then returns left sensor, right sensor and mid sensor
        obstacles = self.obstacle_detection()
        f_obs = [0., 0.]
        k = 0
        for obstacle_distance in obstacles:
            fx, fy = self.obstacle_gaussian(obstacle_distance,
                                            self.sensor_angles[k])
            f_obs[0] += fx
            f_obs[1] += fy
            k += 1

        f_obs[0], f_obs[1] = self.local_to_global(f_obs[0], f_obs[1])
        f_obs[0] *= gains[0]
        f_obs[1] *= gains[1]
        return f_obs

    def robot_neighbors(self):
        """ do some sorting to neighbors and their position """
        neighbors = self.interf.get_neighbors()
        neighbor_ids = []
        neighbor_pos = []
        for neighbor in neighbors:
            neighbor_ids += [neighbor[0]]
            neighbor_pos += [[neighbor[1][0], neighbor[1][1]]]
        return neighbor_ids, neighbor_pos

    def formation_control(self,gains=[1.,1.]):
        """ computes velocity vector due to formation in local
            robot frame then transforms it to global frame """
        fx = 0.
        fy = 0.
        if self.formation.formation is None:
            return 0., 0.
        else:
            nids, npos = self.robot_neighbors()
            id, pos, constraints = self.formation(nids, npos)
            k = 0
            for i in constraints:
                dx, dy = self.local_to_global(pos[k][0], pos[k][1])
                fx += -dx - i[0]
                fy += -dy - i[1]
                k += 1
            lfx = -gains[0] * fx
            lfy = -gains[1] * fy
            return lfx, lfy

    def set_formation(self, formation='Line'):
        self.formation.set_formation(formation)

    def position_control(self, target_pos, position_gains=[1., 1.]):
        """ only works for robot with id = 0 else it returns no control """

        current_pos = self.position()
        distance_to_target = np.sqrt((current_pos[0] - target_pos[0]) ** 2 +
                                     (current_pos[1] - target_pos[1]) ** 2)
        if distance_to_target <= 0.1:  # 10 cm error
            if self.debug: print ' target reached '
            return [0., 0.]
        else:
            fx = position_gains[0] * (target_pos[0] - current_pos[0])
            fy = position_gains[1] * (target_pos[1] - current_pos[1])
            return [fx, fy]

    def keep_formation(self, fx, fy):
        self.integrate_heading()
        if self.id == 5:
            return fx, fy
        else:
            nids, npos = self.robot_neighbors()
            id = self.id+1
            x, y = npos[nids.index(id)]  # get position of robot behind
            d = np.sqrt(x**2 + y**2)
            if abs(d)>1.5:
                return 0., 0.
            else:
                return fx, fy





    def controller(self, target_position, formation, gains):

            _status = True
            fx, fy = 0., 0.
            time.sleep(self.dt)

            if formation is not self.formation.formation:
                # this is really bad should rename it
                self.set_formation(formation)

            lfx, lfy =self.formation_control(gains[0])
            fx += lfx
            fy += lfy

            if self.id == 0:
                # fx, fy = 0., 0.
                f_pos = self.position_control(target_position, gains[1])
                fx += f_pos[0]
                fy += f_pos[1]

            f_obs = self.obstacle_control(gains[2])

            fx += f_obs[0]
            fy += f_obs[1]
            # make sure 0 doesnt escape
            fx, fy = self.keep_formation(fx, fy)

            desired_angle = np.arctan2(fy, fx)
            f = np.sqrt(fx**2 + fy**2)
            #
            # if self.id==0 and formation=='Escape':

            if self.id == 0 and abs(f)<0.02:
                if self.debug: print 'target reached '
                self.interf.set_wheel_velocity([0., 0.])
                self.integrate_heading()
                _status = False

            elif abs(f)<0.15:
                if self.debug: print 'target reached '
                self.interf.set_wheel_velocity([0., 0.])
                self.integrate_heading()
                _status = False

            else:
                # omega resonates sometimes, add kd
                desired_omega = self.kp_omega * (desired_angle
                                                 - self.current_heading) #\
                              #  - self.kd_omega * self.angular_velocity()
                desired_vel = min(self.kp_vel * f, 0.3)
                # desired_omega = min(desired_omega, 1)

                if abs(desired_angle - self.current_heading) <= 0.05:
                    if self.debug: print ' Desired heading reached for robot ', \
                        self.id
                    if self.debug: print 'current angle ', \
                        self.current_heading, ' desired ', desired_angle

                    self.interf.set_wheel_velocity(
                        self.wheel_speeds(desired_vel, 0))
                    self.integrate_heading()

                elif abs(desired_angle - self.current_heading) <= 0.3:
                    self.interf.set_wheel_velocity(self.wheel_speeds(
                        0.5 * desired_vel, 0.5 * desired_omega))
                    self.integrate_heading()
                else:
                    speed = self.wheel_speeds(0.005*desired_vel, desired_omega)
                    speed[0] *= .3
                    speed[1] *= .3
                    self.interf.set_wheel_velocity(speed)
                    # self.interf.set_wheel_velocity(self.wheel_speeds(
                    #     0., desired_omega))
                    self.integrate_heading()

            return _status










def reset_world():
    inter = gazebo_swarm.GazeboWorldIface()
    inter.reset()

if __name__ == "__main__":
    pass
    # reset_world()
    # robot0 = Robot(id=0, dt=.01, debug=True)
    # # robot0.heading_controller(-np.pi)
    #
    # while robot0.controller([2., -1.]):
    #     pass
    # time.sleep(2.0)
    # while robot0.controller([4., -10.]):
    #     pass
    # time.sleep(2.0)
    # while robot0.controller([-10., -7.]):
    #     pass
    # time.sleep(2.0)
    # while robot0.controller([-3., -3.]):
    #     pass
    # time.sleep(2.0)
    # while robot0.controller([-3., 7.]):
    #     pass
    # time.sleep(2.0)
    # while robot0.controller([6., 7.]):
    #     pass
    # time.sleep(2.0)
    # while robot0.controller([7., -9.]):
    #     pass
    # time.sleep(2.0)
    # while robot0.controller([10., -10.]):
    #     pass
