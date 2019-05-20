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

import logging


class positionTask:
    """!@brief
    It generates joint acceleration given a desired position. 
    """

    def __init__(self, skel, desiredPosition, taskWeight, selectionVector=None, Kd=None, Kp=None, bodyNodeIndex=None):

        logger = logging.getLogger(__name__)
        self.robot = skel

        if Kd is None:
            self.Kd = 10
        else:
            self.Kd = Kd

        if Kp is None:
            self.Kp = 10
        else:
            self.Kp = Kp

        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex

        if desiredPosition is None:
            raise Exception("Desired position is not set")
        else:
            self.desiredPosition = desiredPosition


        if taskWeight is None:
            raise Exception("Task weight is not set")
        else:
            self.taskWeight = taskWeight

        if selectionVector is None:
            logger.warning("Selection matrix is identity")
            self.selectionMatrix = np.identity(3)
        else:
            self.selectionMatrix = np.identity(3)
            self.selectionMatrix[0, 0] = selectionVector[0]
            self.selectionMatrix[1, 1] = selectionVector[1]
            self.selectionMatrix[2, 2] = selectionVector[2]

        self.error = np.zeros((3, 1))


        #logger.warning("Kd is: %d", self.Kd)
    def update(self):
        pass


    def calcMatricies(self, useContactVariables, qpContact):

        newJacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian = self.selectionMatrix.dot(newJacobian)

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.selectionMatrix.dot(newJacobian_dot)

        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        translation = transform[[0,1,2],3].reshape((3,1))

        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        self.error = self.selectionMatrix.dot(translation - self.desiredPosition)

        constant = (newJacobian_dot + self.Kd*newJacobian).dot(dq) + self.Kp*(self.error)
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


        return [self.taskWeight*Q, self.taskWeight*P, self.taskWeight*C]
