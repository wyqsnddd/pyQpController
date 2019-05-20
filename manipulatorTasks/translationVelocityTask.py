# Copyright 2018-2019 CNRS-UM LIRMM
#
# \author Yuquan Wang 
#
# 
#
# pyQpController is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# pyQpController is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
# General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with pyQpController. If not, see
# <http://www.gnu.org/licenses/>.


import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array
from qpControllers import qpObj
import logging

class translationVelocityTask:
    def __init__(self, skel, desiredTranslationVelocity, selectionVector=None, Kp=None, bodyNodeIndex=None):

        logger = logging.getLogger(__name__)
        self.robot = skel

        if Kp is None:
            self.Kp = 10
        else:
            self.Kp = Kp

        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex

        if desiredTranslationVelocity is None:
            raise Exception("Desired translationVelocity is not set")
        else:
            self.desiredTranslationVelocity = desiredTranslationVelocity

        if selectionVector is None:
            logger.warning("Selection matrix is identity")
            self.selectionMatrix = np.identity(3)
        else:
            self.selectionMatrix = np.identity(3)
            self.selectionMatrix[0, 0] = selectionVector[0]
            self.selectionMatrix[1, 1] = selectionVector[1]
            self.selectionMatrix[2, 2] = selectionVector[2]

        self.error = np.zeros((3,1))
        self.update()

    def update(self):
        transform = self.robot.bodynodes[-1].world_transform()
        robot_ee_translation = transform[:3, 3]

        self.desiredPosition = robot_ee_translation.reshape((3,1)) + self.desiredTranslationVelocity*self.robot.world.dt

    def calcMatricies(self, useContactVariables, qpContact):

        newJacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian = self.selectionMatrix.dot(newJacobian)

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.selectionMatrix.dot(newJacobian_dot)

        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        self.error = self.selectionMatrix.dot(newJacobian.dot(dq) - self.desiredTranslationVelocity)

        logger = logging.getLogger(__name__)
        logger.debug('The position task error is: %s ', self.error)

        constant = (newJacobian_dot + self.Kp * newJacobian).dot(dq) - self.Kp * self.selectionMatrix.dot(self.desiredTranslationVelocity)

        Q = newJacobian.T.dot(newJacobian)
        Q_size = Q.shape[0]
        Q_new  = np.zeros((Q_size + self.robot.ndofs, Q_size + self.robot.ndofs))
        Q_new[:Q_size, :Q_size] = Q
        Q = Q_new
        
        P = 2*constant.T.dot(newJacobian)
        zero_vector = np.zeros((1, self.robot.ndofs))
        P = np.concatenate((P, zero_vector), axis=1)

        C = constant.T.dot(constant)

        if(useContactVariables):
            QP_size = 2*self.robot.ndofs
            contact_size = qpContact.Nc
            Q_new  = np.zeros((QP_size + contact_size, QP_size + contact_size))
            Q_new[:QP_size, :QP_size] = Q # write the old info
            Q = Q_new

            P_new = np.zeros((1, QP_size + contact_size))
            P_new[0, :QP_size] = P
            P = P_new

        return [Q, P, C]



