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

    def calcMatricies(self, useContactVariables, qpContact):

        jacobian = self.robot.bodynodes[-1].linear_jacobian()
        M_inv = np.linalg.pinv(self.robot.M)
        temp = np.linalg.pinv(jacobian.dot(M_inv).dot(jacobian.transpose()) )
        J_dagger = jacobian.transpose().dot(temp)

        
        #M_inv = np.linalg.pinv(self.robot.M)
        # jacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        #jacobian = self.robot.bodynodes[self.bodyNodeIndex].jacobian()
        #temp_1 = M_inv.dot(jacobian.T)
        #temp_2 = np.linalg.pinv(jacobian.dot(temp_1))
        #constant = temp_1.dot(temp_2)

        dq =  (self.robot.dq).reshape((self.robot.ndofs, 1))

        # temp_A = (self.res + 1)*constant.dot(jacobian)
        temp_A = (self.res + 1)*(M_inv.dot(J_dagger)).dot(jacobian)
        #temp_A = (-self.res + 1)*(M_inv.dot(J_dagger)).dot(jacobian)
        # temp_A = (self.res + 1)*(self.robot.M)

        A = np.block([
            [temp_A*self.robot.world.dt, np.identity(self.robot.ndofs)]
        ])

        # A = np.block([
        #     [np.zeros((6,6)), np.zeros((6,6))]
        # ])
        

        
        b  = -temp_A.dot(dq)


        if(useContactVariables):
            # Append more columns corresponding to the contact force varialbes
            contact_size = qpContact.Nc
            row_number = A.shape[0]
            column_number = A.shape[1]
            A_new = np.zeros((row_number, column_number + contact_size))
            A_new[:, :column_number] = A  # write the old info
            A = A_new

        
        return [A, b]
