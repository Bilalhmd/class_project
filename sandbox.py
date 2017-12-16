import gazebo_swarm
import time
import numpy as np

# wheel radius = 0.033
# distance between wheels L = 0.26




def reset_world():
    inter = gazebo_swarm.GazeboWorldIface()
    inter.reset()

# reset_world()








def constraint_controller():
    cmat = np.array([])



def get_Edges(interface):
    # if robot 1 can talk to 2 then 2 can talk to 1
    """ loops over all robots, finds neighbors per robot, constructs list of all
        neighbors per robot
        then iterates over all connections to remove repeated ones, finally it
        constructs edges in pairs
    """
    conn=[]
    edges=[]
    for vertex in interface:
        robs =[]  # list of neighbor robot ids
        neighbors = vertex.get_neighbors()   # find out what robots can be reached
                                             # by each robot

        # create a list of neighbor robot Ids
        for rob in neighbors:
            robs += [rob[0]]
        conn += [robs]


    init_con = conn
    print conn
    # iterate over all connections remove repeated ones (reverted)
    for i in conn:
        b = i
        print 'i = ', i
        for k in b:
            del conn[k][0]

    # construct edges
    for k in range(len(conn)-1):
        for j in conn[k]: # robot j available connections
            edges += [[k,j],]



    return init_con, conn, edges



def get_Laplacian(E,n_vertex):
    L = np.zeros([n_vertex,n_vertex]) #our Laplacian matrix
    Delta = np.zeros([n_vertex,n_vertex]) #this is the degree matrix
    A = np.zeros([n_vertex,n_vertex]) #this is the adjacency matrix
    for e in E: #for each edge in E
        #add degrees
        Delta[e[1],e[1]] +=1
        #add the input in the adjacency matrix
        A[e[1],e[0]] = 1
        #symmetric connection as we have undirected graphs
        Delta[e[0],e[0]] +=1
        A[e[0],e[1]] = 1
    L = Delta - A
    return L



# get incidence matrix for directed graph E (list of edges)
def getIncidenceMatrix(E,n_vertex):
    n_e = len(E)
    D = np.zeros([n_vertex,n_e])
    for e in range(n_e):
        #add the directed connection
        D[E[e][0],e] = -1
        D[E[e][1],e] = 1
    return D




interf = []
for i in range(6):
    interf += [gazebo_swarm.GazeboRobotIface(i)]


init_con, conn, E = get_Edges(interf)

print 'Connections: \n', init_con


print 'List of Edges: \n', E

L = get_Laplacian(E, 6)

print 'Laplacian: \n', L

D = getIncidenceMatrix(E, 6)

print 'Incidence Matrix: \n', D






def formation_control(robot_num, interface):

    ## start with formation control
    neighbors = interface.get_neighbors()

    desired_vel = [0., 0.]
    des_vel = np.array([0., 0.])

    for n in neighbors:
        pass





def robotzero_controller(interface, target):
    current_position = interface.get_gps()
    des_vel = np.array([0.,0.])
    desired_vel = [0., 0.]
    des_vel[0] = 5 * (target[0] - current_position[0])
    des_vel[1] = 5 * (target[1] - current_position[1])

    theta_des = np.arctan2(des_vel[1], des_vel[0])

    desired_vel[0] = des_vel[0] - des_vel[1]
    desired_vel[1] = des_vel[0] + des_vel[1]


    interface.set_wheel_velocity(desired_vel)


def robotzero_controller2(interface, target, theta):
    current_position = interface.get_gps()

    direction = np.arctan2(target[1]-current_position[1],target[0]-current_position[0])

    des_vel = np.array([0., 0.])
    desired_vel = [0., 0.]
    des_vel[0] = 5 * (target[0] - current_position[0])
    des_vel[1] = 5 * (target[1] - current_position[1])

    theta_des = np.arctan2(des_vel[1], des_vel[0])

    desired_vel[0] = des_vel[0] - des_vel[1]
    desired_vel[1] = des_vel[0] + des_vel[1]

    interface.set_wheel_velocity(desired_vel)


def test_rotation(interface, current_angle, desired_angle):
    if current_angle<=desired_angle:
        interface.set_wheel_velocity([-1.,1.])
        return True
    else:
        print 'current angle ', current_angle, ' desired ', desired_angle
        interface.set_wheel_velocity([0., 0.])
        return False

radius = 0.033
distance = 0.15
omega = radius*2.0/distance
print omega
desired_theta = np.pi
dt = .001
robot0_theta = 0.0



# while (True):
#     robotzero_controller(interf[0], [.5, 0.])
#     # test_rotation(interf[0], robot0_theta, desired_theta)
#
#     if interf[0].get_gps()[0] >= .485: break
#
#     time.sleep(dt)





while test_rotation(interf[0], robot0_theta, desired_theta):
    robot0_theta += dt * omega
    time.sleep(dt)  # 10 ms sleep




# def rotation_control(self, desired_angle, current_angle):
#     if desired_angle>=0.:
#         if current_angle <= desired_angle:
#             self.interf.set_wheel_velocity([-1., 1.])
#             omega = self.r * 2. / self.l
#             return True, omega
#         else:
#             print 'current angle ', self.current_heading, ' desired ', desired_angle
#             self.interf.set_wheel_velocity([0., 0.])
#             return False, 0.
#     else:
#         if desired_angle<=current_angle:
#             self.interf.set_wheel_velocity([1., -1.])
#             omega = -self.r * 2. / self.l
#             return True, omega
#         else:
#             print 'current angle ', self.current_heading, ' desired ', desired_angle
#             self.interf.set_wheel_velocity([0., 0.])
#             return False, 0.

