import numpy as np




class SwarmFormation(object):
    """ create an object describing the graph and
        the formation of the swarm network locally """
    def __init__(self, interf, id):
        self.bot = interf
        self.formation = None
        self.id = id

    def set_formation(self, formation='Line'):
        self.formation = formation
        print ' Formation changed to ', formation

    def __call__(self, nids, npos):
        if self.formation == 'Line':
            return self.line_formation(nids, npos,
                                       dpos=.75, alpha=np.pi/2)
        elif self.formation =='Consensus':
            return self.consensus_protocol(nids, npos)
        elif self.formation == 'Escape':
            return self.escape_tunnel(nids, npos,
                     dpos=.5, alpha=np.pi/2)
            # return self.line_formation(nids, npos,
            #                            dpos=.5, alpha=np.pi / 2)


        else:
            return None

    def consensus_protocol(self, nids, npos):
        constraints = []
        for nid in nids:
            # if nid<self.id:
            constraints += [[0., 0.]]
        return constraints

    def line_formation(self, nids, npos,
                       dpos=1., alpha=np.pi/2):
        """straight line along direction alpha with spacing dpos
            returns set of constraints per bot depending on
            neighbor positions """
        dx = dpos * np.cos(alpha)
        dy = dpos * np.sin(alpha)
        constraints = []
        for nid in nids:
            constraints += [[dx * (self.id - nid),
                             dy * (self.id - nid)]]
        return nids, npos, constraints

    def escape_tunnel(self, nids, npos,
                      dpos=.75, alpha=np.pi/2):
        dx = dpos * np.cos(alpha)
        dy = dpos * np.sin(alpha)
        constraints = []
        ids =[]
        pos = []
        k=0
        for nid in nids:

            if self.id == 0 and nid == 1:
                ids += [nid]
                pos += [npos[k]]
                constraints += [[dx * (self.id - nid),
                                 dy * (self.id - nid)]]
                k += 1

            elif self.id == 0 and nid == 2:
                ids += [nid]
                pos += [npos[k]]
                constraints += [[dx * (self.id - nid),
                                 dy * (self.id - nid)]]
                k += 1

            elif nid<self.id:
                ids += [nid]
                pos += [npos[k]]
                constraints += [[dx * (self.id - nid),
                                 dy * (self.id - nid)]]
                k+=1

        return ids, pos, constraints

    def box(self, nids, npos, dpos=.5, alpha=np.pi/2):
        graph =[[1, 2],
                [0, 2, 3],
                [0, 1, 3, 4],
                [2, 4, 5],
                [2, 3, 5],
                [3, 4]]
        graph = graph[self.id]
        switch


    def box_formation_horizontal(self, dx=1., dy=1.):
        constraints = []
        return constraints

    def box_formation_vertical(self, dx=1., dy=1.):
        constraints = []
        return constraints



