import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array

import logging

from collections import namedtuple

class jointVelocityJumpEstimator:
    """!@brief
    It predicts the upper and lower bounds of the joint velocities jump under impacts
    This function needs to be called after we generated the optimal joint acceleration and before it is applied to the robot.
    """


    def __init__(self, skel, resLower = 0.4, resUpper = 0.6, bodyNodeIndex = -1):
        self.robot = skel
        self.bodyNodeIndex = bodyNodeIndex

        self.initializeLog()

        self.resLower = resLower
        self.resUpper = resUpper

        self.positionUpper = (self.robot.position_upper_limits()).reshape((self.robot.ndofs, 1))
        self.positionLower = (self.robot.position_lower_limits()).reshape((self.robot.ndofs, 1))

        self.velocityUpper = []
        self.velocityLower = []
        for ii in range(0, self.robot.ndofs):
            dof = self.robot.dof(ii)
            self.velocityUpper.append(dof.velocity_upper_limit())
            self.velocityLower.append(dof.velocity_lower_limit())

        self.velocityUpper = np.reshape(self.velocityUpper, (self.robot.ndofs, 1))
        self.velocityLower = np.reshape(self.velocityLower, (self.robot.ndofs, 1))

        self.dt = self.robot.world.dt

    def initializeLog(self):

        self.predictionLog = namedtuple('log', ['delta_dq_lower','delta_dq_upper', 'ddqUpperBoundPosition', 'ddqLowerBoundPosition','ddqUpperBoundVelocity', 'ddqLowerBoundVelocity'])
        self.time = []
        self.predictionLog.deltaDqLower = []
        self.predictionLog.deltaDqUpper = []
        self.predictionLog.ddqUpperBoundPosition = []
        self.predictionLog.ddqLowerBoundPosition = []
        self.predictionLog.ddqUpperBoundVelocity = []
        self.predictionLog.ddqLowerBoundVelocity = []

    def saveLog(self, predictDeltaDqLower, predictDeltaDqUpper,ddqLowerPosition, ddqUpperPosition, ddqLowerVelocity, ddqUpperVelocity):

        self.time.append(self.robot.world.t)

        self.predictionLog.deltaDqLower.append(predictDeltaDqLower)
        self.predictionLog.deltaDqUpper.append(predictDeltaDqUpper)
        self.predictionLog.ddqUpperBoundPosition.append(ddqUpperPosition)
        self.predictionLog.ddqLowerBoundPosition.append(ddqLowerPosition)
        self.predictionLog.ddqUpperBoundVelocity.append(ddqUpperVelocity)
        self.predictionLog.ddqLowerBoundVelocity.append(ddqLowerVelocity)

    def calcImpulsiveQuantities(self, sol_ddq):

        M_inv = np.linalg.pinv(self.robot.M)

        jacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()

        temp_1 = M_inv.dot(jacobian.T)

        temp_2 = np.linalg.pinv(jacobian.dot(temp_1))

        constant = temp_1.dot(temp_2)

        dq =  (self.robot.dq).reshape((self.robot.ndofs, 1))


        baseOne = constant.dot(jacobian.dot( dq + self.dt*sol_ddq   ))

        predicted_delta_dq_upper = self.resUpper*baseOne
        predicted_delta_dq_lower = self.resLower*baseOne

        ddq = (self.robot.ddq).reshape((self.robot.ndofs, 1))
        position = (self.robot.q).reshape((self.robot.ndofs, 1))


        ddq_upper_bound_position = (self.positionUpper - position + ddq*(self.dt*self.dt) + predicted_delta_dq_upper*self.dt)/(self.dt*self.dt)
        ddq_lower_bound_position = (self.positionLower - position + ddq * (self.dt*self.dt) + predicted_delta_dq_upper * self.dt) / (
        self.dt*self.dt)

        ddq_upper_bound_Velocity = -(self.velocityLower - dq - ddq * self.dt - predicted_delta_dq_upper) / self.dt
        ddq_lower_bound_Velocity = -(self.velocityUpper - dq - ddq*self.dt - predicted_delta_dq_upper)/self.dt

        return [predicted_delta_dq_lower, predicted_delta_dq_upper, ddq_lower_bound_position, ddq_upper_bound_position, ddq_lower_bound_Velocity, ddq_upper_bound_Velocity]


    def update(self, sol_ddq):

        [predicted_delta_dq_lower, predicted_delta_dq_upper, ddq_lower_bound_position, ddq_upper_bound_position, ddq_lower_bound_Velocity, ddq_upper_bound_Velocity] = self.calcImpulsiveQuantities(sol_ddq)


        self.saveLog(predicted_delta_dq_lower, predicted_delta_dq_upper, ddq_lower_bound_position, ddq_upper_bound_position, ddq_lower_bound_Velocity, ddq_upper_bound_Velocity)

