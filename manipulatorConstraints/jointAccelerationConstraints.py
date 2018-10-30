import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array

from cvxopt import matrix, solvers

class jointAccelerationLimitConstraints:
    """!@brief
        defined for acc variables
        We generate right hand side 'B' for constraints in the form: Ax <= B
    """
    def __init__(self, skel):
        self.robot = skel

        self.upper = 5*np.array([0.87266, 0.87266, 0.87266, 1.466, 2.059, 2.094])
        self.lower = self.upper


    def update(self):
        pass

    def calcMatricies(self):

        G = np.concatenate((np.identity(self.robot.ndofs), -np.identity(self.robot.ndofs)), axis=0)
        h = np.reshape(np.concatenate((self.upper, self.lower)),(12,1))

        return [G, h]

if __name__ == "__main__":

    print('Hello, PyDART!')



    pydart.init()

    test_world = pydart.World(1.0 / 2000.0, "../data/skel/two_cubes.skel")

    test_robot = test_world.add_skeleton("../data/KR5/KR5_sixx_R650.urdf")

    #upper = np.reshape(array([1.0, 1.0, 1.0]), (3,1))
    dt = 0.1
    #lower = np.reshape(array([-1.0, -1.0, -1.0]), (3,1))

    a = jointAccelerationLimitConstraints(test_robot)


    [G, h ] = a.calcMatricies()
    print "The G  is: ",'\n', G, G.shape
    print "The h is: ",'\n', h, h.shape



