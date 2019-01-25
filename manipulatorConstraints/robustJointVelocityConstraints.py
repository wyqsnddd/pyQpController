import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array

from cvxopt import matrix, solvers

class robustJointVelocityLimitConstraints:
    """!@brief
    We use Euler forward to approximate the derivative and introduce the impact robust joint velocity constraints. 
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

        self.upper = []
        self.lower = []
        for ii in range(0, self.robot.ndofs):
            dof = self.robot.dof(ii)
            self.upper.append(dof.velocity_upper_limit())
            self.lower.append(dof.velocity_lower_limit())

        print "The joint velocity upper limit is: ", self.upper
        print "The joint velocity lower limit is: ", self.lower
        
        self.upper = np.reshape(self.upper, (self.robot.ndofs, 1))
        self.lower = np.reshape(self.lower, (self.robot.ndofs, 1))

        self.dt = dt

    def upperRhs(self, dq):
        return  (self.upper - dq)

    def update(self, impactEstimator):
        pass

    def lowerRhs(self, dq):
        return    -(self.lower - dq)


    def calcMatricies(self):
        zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))

        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        G = np.concatenate((np.identity(self.robot.ndofs), -np.identity(self.robot.ndofs)), axis=0)
        G = np.concatenate((zero_block, G), axis=1)
        h = np.concatenate(( self.upperRhs(dq), self.lowerRhs(dq)), axis=0)

        return [G, h]

if __name__ == "__main__":

    print('Hello, PyDART!')



    pydart.init()

    test_world = pydart.World(1.0 / 2000.0, "../data/skel/two_cubes.skel")

    test_robot = test_world.add_skeleton("../data/KR5/KR5_sixx_R650.urdf")

    #upper = np.reshape(array([1.0, 1.0, 1.0]), (3,1))
    dt = 0.1
    #lower = np.reshape(array([-1.0, -1.0, -1.0]), (3,1))

    a = robustJointVelocityLimitConstraints(test_robot, dt)


    [G, h ] = a.calcMatricies()
    print "The G  is: ",'\n', G, G.shape
    print "The h is: ",'\n', h, h.shape


    w, v = np.linalg.eig(test_robot.M)
    print "w", w.shape
    print "v", v.shape
    print "1/w", 1/w
    print "w", w
    print "test", test_robot.M.dot((v.dot(np.diag(1/w)).dot(v.T)))
