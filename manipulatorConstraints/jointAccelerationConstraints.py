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
        self.upper = np.reshape(self.upper, (self.robot.ndofs, 1))
        self.lower = -self.upper
        
        print "The joint acc upper limit is: ", self.upper
        print "The joint acc lower limit is: ", self.lower

    def update(self, impactEstimator):
        self.average_impact_acc = impactEstimator.readAverageDdq()
        self.average_impact_acc = np.reshape(self.average_impact_acc, (self.robot.ndofs, 1))
        
    def calcMatricies(self):
        zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))

        G_1 = np.concatenate((np.identity(self.robot.ndofs), -np.identity(self.robot.ndofs)), axis=0)
        G_2 = np.concatenate((np.identity(self.robot.ndofs), -np.identity(self.robot.ndofs)), axis=0)

        # G = np.concatenate((G_1, G_2), axis=0)
        G = G_1
        G = np.concatenate((G, zero_block), axis=1)

        h_1 = np.reshape(np.concatenate((self.upper, -self.lower)),(12,1))
        #h_2 = np.reshape(np.concatenate((self.upper - self.average_impact_acc, -self.lower + self.average_impact_acc)),(12, 1))
                         
        # h = np.reshape(np.concatenate((h_1, h_2)), (24,1))
        h = h_1
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



