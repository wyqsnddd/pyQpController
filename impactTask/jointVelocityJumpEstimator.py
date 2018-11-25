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

        self.dt = self.robot.world.dt

    def initializeLog(self):

        self.predictionLog = namedtuple('log', ['delta_dq_lower','delta_dq_upper'])
        self.time = []
        self.predictionLog.deltaDqLower = []
        self.predictionLog.deltaDqUpper = []

    def saveLog(self, predictDeltaDqLower, predictDeltaDqUpper):

        self.time.append(self.robot.world.t)

        self.predictionLog.deltaDqLower.append(predictDeltaDqLower)
        self.predictionLog.deltaDqUpper.append(predictDeltaDqUpper)



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

        return [predicted_delta_dq_lower, predicted_delta_dq_upper]


    def update(self, sol_ddq):

        [predicted_delta_dq_lower, predicted_delta_dq_upper] = self.calcImpulsiveQuantities(sol_ddq)


        self.saveLog(predicted_delta_dq_lower, predicted_delta_dq_upper)

