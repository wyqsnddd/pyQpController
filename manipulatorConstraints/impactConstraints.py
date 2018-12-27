import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array


class impactConstraints:

    def __init__(self, robot, res = 1.0, bodyNodeIndex=None):
        self.robot = robot
        self.dof = self.robot.ndofs
        self.res = res
        
        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex


    def update(self, impactEstimator):
        pass

    def calcMatricies(self):
        M_inv = np.linalg.pinv(self.robot.M)
        jacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        temp_1 = M_inv.dot(jacobian.T)
        temp_2 = np.linalg.pinv(jacobian.dot(temp_1))
        constant = temp_1.dot(temp_2)

        dq =  (self.robot.dq).reshape((self.robot.ndofs, 1))

        temp_A = (self.res + 1)*constant.dot(jacobian)

        A = np.block([
            [temp_A*self.robot.world.dt, np.identity(self.robot.ndofs)]
        ])
        # A = np.block([
        #     [np.zeros((6,6)), np.zeros((6,6))]
        # ])
        

        
        b  = -temp_A.dot(dq)
        
        return [A, b]
