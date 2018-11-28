import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array

from cvxopt import matrix, solvers

class jointLimitConstraints:
    """!@brief
        defined for acc variables
        We generate right hand side 'B' for constraints in the form: Ax <= B
    """

    def __init__(self, skel, dt):
        self.robot = skel

        # if len(upper) != len(lower):
        #     raise Exception("Dimension does not match")
        # if upper.shape[1] is not 1:
        #     raise Exception("Upper limits is not a vector")
        # if lower.shape[1] is not 1:
        #     raise Exception("Lower limits is not a vector")

        if ((dt<0.0) and (dt > 1.0)):
            raise Exception("Unproper dt")

        self.upper = (self.robot.position_upper_limits()).reshape((self.robot.ndofs, 1))
        self.lower = (self.robot.position_lower_limits()).reshape((self.robot.ndofs, 1))
        print "The joint upper limit is: ", self.upper.transpose()
        print "The joint lower limit is: ", self.lower.transpose()

        self.dt = dt

    def upperRhs(self, q, dq):
        return (self.upper - q)*(1/(self.dt*self.dt)) - dq*(1/self.dt)

    def update(self, impactEstimator):
        pass

    def lowerRhs(self, q, dq):
        return -((self.lower - q)*(1/(self.dt*self.dt)) - dq*(1/self.dt))

    def calcMatricies(self):
        zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))
        
        q = (self.robot.q).reshape((self.robot.ndofs,1))
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        G = np.concatenate((np.identity(self.robot.ndofs), -np.identity(self.robot.ndofs)), axis=0)
        G = np.concatenate((G, zero_block), axis=1)
        h = np.concatenate(( self.upperRhs(q, dq), self.lowerRhs(q, dq)), axis=0)

        return [G, h]


if __name__ == "__main__":

    print('Hello, PyDART!')

    pydart.init()

    test_world = pydart.World(1.0 / 2000.0, "../data/skel/two_cubes.skel")

    test_robot = test_world.add_skeleton("../data/KR5/KR5_sixx_R650.urdf")

    #upper = np.reshape(array([1.0, 1.0, 1.0]), (3,1))
    dt = 0.1
    #lower = np.reshape(array([-1.0, -1.0, -1.0]), (3,1))

    a = jointLimitConstraints(test_robot, dt)


    [G, h ] = a.calcMatricies()
    print "The G  is: ",'\n', G, G.shape
    print "The h is: ",'\n', h, h.shape

    # dim = 3
    # test_q = uniform(dim,1)
    # test_dq = uniform(dim,1)
    #
    # print "rhs of upper limit constraint is: ", a.upperRhs(test_q, test_dq )
    # print "rhs of lower limit constraint is: ", a.lowerRhs(test_q, test_dq)
    # print test_dq.size
    # test_x = array([[1, 0]])
    #
    # print np.reshape (test_x.shape, (2,1)).shape[1]
    #
