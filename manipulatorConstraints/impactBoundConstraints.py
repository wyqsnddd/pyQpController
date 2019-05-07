import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array


class impactBoundConstraints:

    # This is an inequality implementation of the constraints that link joint velocity jump and joint accelerations.
    
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
        jacobian = self.robot.bodynodes[-1].linear_jacobian()
        M_inv = np.linalg.pinv(self.robot.M)
        temp = np.linalg.pinv(jacobian.dot(M_inv).dot(jacobian.transpose()) )
        J_dagger = jacobian.transpose().dot(temp)
        
        # predict_impulse_tau = J_dagger.dot(predicted_delta_dq_upper)/self.dt

        # M_inv = np.linalg.pinv(self.robot.M)
        # jacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        # temp_1 = M_inv.dot(jacobian.T)
        # temp_2 = np.linalg.pinv(jacobian.dot(temp_1))
        # constant = temp_1.dot(temp_2)

        dq =  (self.robot.dq).reshape((self.robot.ndofs, 1))
        # temp_A = (self.res + 1)*constant.dot(jacobian)
        temp_A_upp = (self.res + 1)*(M_inv.dot(J_dagger)).dot(jacobian)
        temp_A_low = (self.res - 1)*(M_inv.dot(J_dagger)).dot(jacobian)
        # temp_A_upp = (self.res + 1)*(self.robot.M)
        # temp_A_low = (-self.res + 1)*(self.robot.M)

        # G_upp = np.block([
        #     [temp_A_upp*self.robot.world.dt, np.identity(self.robot.ndofs)]
        # ])

        G_upp  = np.zeros((temp_A_upp.shape[0], temp_A_upp.shape[1] + self.robot.ndofs))
        G_upp[:temp_A_upp.shape[0], :temp_A_upp.shape[1]] = temp_A_upp*self.robot.world.dt
        G_upp[:temp_A_upp.shape[0], temp_A_upp.shape[1]:] = np.identity(self.robot.ndofs)

        
        # G_low = np.block([
        #     [temp_A_low*self.robot.world.dt, -np.identity(self.robot.ndofs)]
        # ])
        G_low  = np.zeros((temp_A_low.shape[0], temp_A_low.shape[1] + self.robot.ndofs))
        G_low[:temp_A_low.shape[0], :temp_A_low.shape[1]] = temp_A_low*self.robot.world.dt
        G_low[:temp_A_low.shape[0], temp_A_low.shape[1]:] = -np.identity(self.robot.ndofs)


        G = np.concatenate((G_upp, G_low), axis=0)
        h_upp  = -temp_A_upp.dot(dq)
        h_low = -temp_A_low.dot(dq)
        return [G, np.concatenate((h_upp, h_low), axis=0)]
