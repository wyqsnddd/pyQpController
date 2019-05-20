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

from worlds import cubes_KR_R650
from controllers import gravityCompensationController
from qpControllers import qpObj

import logging


class orientationTask:
    """!@brief
    We use unit quaternion and second order inverse dynamics to fulfill an orientation task
    """

    def __init__(self, skel, desiredOrientation, taskWeight, scalarWeight, selectionVector=None, Kd=None, Kp=None, bodyNodeIndex=None):

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

        if desiredOrientation is None:
            raise Exception("Desired orientation is not set")
        else:
            self.desiredOrientation = desiredOrientation

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

        self.quat_desired_m = self.Q_bar(self.desiredOrientation)

        self.q_scalar_weight = scalarWeight
        # construct the quaternion multiplication matrix:

        # Error checking:
        self.current_quat_d_last = np.zeros((4,1))
        self.current_quat_dd_last = np.zeros((4, 1))

        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        rotation = transform[:3,:3].reshape((3,3))
        self.iniRotation = rotation
        self.rotation_last = self.iniRotation

        self.error_last = np.zeros((4,1))
        self.error_v_last = np.zeros((4,1))

        
        self.current_quat_last = pydart.utils.transformations.quaternion_from_matrix(rotation)

    def Q(self, quaternion):

        Q = np.identity(4)

        Q[0, 0] = quaternion[0]
        Q[1, 0] = quaternion[1]
        Q[2, 0] = quaternion[2]
        Q[3, 0] = quaternion[3]

        Q[0, 1] = -quaternion[1]
        Q[1, 1] = quaternion[0]
        Q[2, 1] = -quaternion[3]
        Q[3, 1] = quaternion[2]

        Q[0, 2] = -quaternion[2]
        Q[1, 2] = quaternion[3]
        Q[2, 2] = quaternion[0]
        Q[3, 2] = -quaternion[1]

        Q[0, 3] = -quaternion[3]
        Q[1, 3] = -quaternion[2]
        Q[2, 3] = quaternion[1]
        Q[3, 3] = quaternion[0]

        return Q

    def Q_bar(self,quaternion):

        Q_bar = np.identity(4)

        Q_bar[0, 0] = quaternion[0]
        Q_bar[1, 0] = -quaternion[1]
        Q_bar[2, 0] = -quaternion[2]
        Q_bar[3, 0] = -quaternion[3]

        Q_bar[0, 1] = quaternion[1]
        Q_bar[1, 1] = quaternion[0]
        Q_bar[2, 1] = quaternion[3]
        Q_bar[3, 1] = -quaternion[2]

        Q_bar[0, 2] = quaternion[2]
        Q_bar[1, 2] = -quaternion[3]
        Q_bar[2, 2] = quaternion[0]
        Q_bar[3, 2] = quaternion[1]

        Q_bar[0, 3] = quaternion[3]
        Q_bar[1, 3] = quaternion[2]
        Q_bar[2, 3] = -quaternion[1]
        Q_bar[3, 3] = quaternion[0]
        return Q_bar


    def W(self, quaternion):

        quat_W = np.zeros([3,4])

        quat_W[0, 0] = -quaternion[1]
        quat_W[1, 0] = -quaternion[2]
        quat_W[2, 0] = -quaternion[3]

        quat_W[0, 1] = quaternion[0]
        quat_W[1, 1] = quaternion[3]
        quat_W[2, 1] = -quaternion[2]

        quat_W[0, 2] = -quaternion[3]
        quat_W[1, 2] = quaternion[0]
        quat_W[2, 2] = quaternion[1]

        quat_W[0, 3] = quaternion[2]
        quat_W[1, 3] = -quaternion[1]
        quat_W[2, 3] = quaternion[0]

        return quat_W


    def W_prime(self, quaternion):

        quat_W_prime = np.zeros([3, 4])

        quat_W_prime[0, 0] = quaternion[1]
        quat_W_prime[1, 0] = quaternion[2]
        quat_W_prime[2, 0] = quaternion[3]

        quat_W_prime[0, 1] = quaternion[0]
        quat_W_prime[1, 1] = -quaternion[3]
        quat_W_prime[2, 1] = quaternion[2]

        quat_W_prime[0, 2] = quaternion[3]
        quat_W_prime[1, 2] = quaternion[0]
        quat_W_prime[2, 2] = -quaternion[1]

        quat_W_prime[0, 3] = -quaternion[2]
        quat_W_prime[1, 3] = quaternion[1]
        quat_W_prime[2, 3] = quaternion[0]

        return quat_W_prime

    def update(self):
        pass

    def vee(self, input):
        w = np.zeros((3,1))
        w[0] = 0.5*(input[2,1] - input[1, 2])
        w[1] = 0.5*(-input[2,0] + input[0, 2])
        w[2] = 0.5*(input[1,0] - input[0, 1])
        
        return w
    
    def calcMatricies(self, useContactVariables, qpContact):

        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()

        rotation = transform[:3,:3].reshape((3,3))

        current_quat = pydart.utils.transformations.quaternion_from_matrix(rotation)
        current_quat = np.reshape(current_quat, (4,1))

        deletion_matrix = np.zeros([3,4])
        deletion_matrix[:,1:] = np.identity(3)

        block_one = deletion_matrix.dot(self.quat_desired_m.dot(self.W(current_quat).transpose()))

        angular_jacobian = self.robot.bodynodes[self.bodyNodeIndex].angular_jacobian()
        angular_jacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].angular_jacobian_deriv()
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        newJacobian = 0.5*block_one.dot(angular_jacobian)
        newJacobian = self.selectionMatrix.dot(newJacobian)



        error = self.quat_desired_m.dot(current_quat)

        error = np.reshape(error, (4, 1))
        #print ("The desired quaternion is: ", '\n', self.desiredOrientation)
        #print ("The current quaternion is: ", '\n', current_quat.transpose())

        #print ("The orientation task error is: ", '\n', error.transpose())
        error = self.selectionMatrix.dot(deletion_matrix.dot(error))

        constant = (0.5 * block_one.dot(angular_jacobian_dot + self.Kd * angular_jacobian)).dot(dq) + self.Kp * (error)

        Q = newJacobian.T.dot(newJacobian)

        Q_size = Q.shape[0]
        Q_new  = np.zeros((Q_size + self.robot.ndofs, Q_size + self.robot.ndofs))
        Q_new[:Q_size, :Q_size] = Q
        Q = Q_new

        
        P = 2 * constant.T.dot(newJacobian)
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


