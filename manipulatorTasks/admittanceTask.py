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
from contact import qpContact

import logging


class admittanceTask:
    """!@brief
    @class admittanceTask
    It generates a desired force with joint acceleration.
    """
    def __init__(self,skel, desiredForce, weight, selectionVector=None, velocityControl=False, desiredVelocity=None, Kd=None, Kf=None, Ki = None, bodyNodeIndex=None, qpForceRegulating=True):
        logger = logging.getLogger(__name__)

        self.robot = skel

        if Kd is None:
            self.Kd = 10
        else:
            self.Kd = Kd

        if Kf is None:
            self.Kf = 10
        else:
            self.Kf = Kf

        if Ki is None:
            self.Ki = 100
        else:
            self.Ki = Ki

        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex

        if desiredForce is None:
            raise Exception("Desired force is not set")
        else:
            self.desiredForce = desiredForce

        if weight is None:
            raise Exception("task weight is not set")
        else:
            self.weight = weight
            
        if selectionVector is None:
            logger.warning("Selection matrix is identity")
            self.selectionMatrix = np.identity(3)
        else:
            self.selectionMatrix = np.identity(3)
            self.selectionMatrix[0, 0] = selectionVector[0]
            self.selectionMatrix[1, 1] = selectionVector[1]
            self.selectionMatrix[2, 2] = selectionVector[2]

            

        self.velocityControl = velocityControl

        if desiredVelocity is None:
            self.desiredVelocity = array([0.0, 0.0, 0.0]).reshape((3,1))
        else:
            self.desiredVelocity = desiredVelocity


        self.contactCheck = False
        self.error = np.zeros((3, 1))
        self.forceErrorIntegral = np.zeros((3, 1))
        self.qpForceErrorIntegral = np.zeros((3, 1))
        self.equivalentForceVector = np.zeros((3, 1))

        # If we should use QP force to regulate the contact force task or not
        self.qpForceRegulating = qpForceRegulating


    def update(self):
        pass

    def calcQPAdmittaceMatricies(self, qpContact):

        qp_Ki = 10
        qp_Kf = 0.5

        newJacobian_linear = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian_linear = self.selectionMatrix.dot(newJacobian_linear)

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.selectionMatrix.dot(newJacobian_dot)

        # Calculating the equivalent end-effector force
        equivalentForce = np.linalg.pinv(newJacobian_linear.T).dot(self.robot.constraint_forces())

        f_qp = qpContact.getContactForce()
        qpForceError = self.selectionMatrix.dot(f_qp - self.desiredForce)
        
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        constant = newJacobian_dot.dot(dq) + qp_Kf * qpForceError + qp_Ki*self.qpForceErrorIntegral

        self.qpForceErrorIntegral = self.qpForceErrorIntegral + qp_Ki*qpForceError*self.robot.world.dt

        Q = newJacobian_linear.T.dot(newJacobian_linear)
        Q_size = Q.shape[0]
        Q_new  = np.zeros((Q_size + self.robot.ndofs, Q_size + self.robot.ndofs))
        Q_new[:Q_size, :Q_size] = Q
        Q = Q_new

        P = 2 * constant.T.dot(newJacobian_linear)
        zero_vector = np.zeros((1, self.robot.ndofs))
        P = np.concatenate((P, zero_vector), axis=1)

        C = constant.T.dot(constant)

        return [Q, P, C]

    def calcQPRegulatingMatricies(self, qpContact):
        # Minimize the error: ||f_qp - f_desired||_2
        constant = 1.0
        f_qp = qpContact.getContactForce()

        force_Q = np.identity(4)

        force_P = - constant*(self.desiredForce.transpose()).dot(qpContact.getContactGenerationMatrix())

        C = constant*self.desiredForce.transpose().dot(self.desiredForce)

        return [force_Q, force_P, C]


    def basicCase(self, useContactVariables, qpContact):
        #newJacobian = self.robot.bodynodes[self.bodyNodeIndex].world_jacobian()
        newJacobian_linear = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian_linear = self.selectionMatrix.dot(newJacobian_linear)

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.selectionMatrix.dot(newJacobian_dot)

        # Calculating the equivalent end-effector force
        equivalentForce = np.linalg.pinv(newJacobian_linear.T).dot(self.robot.constraint_forces())
        


        self.equivalentForceVector = - equivalentForce[0:3:1].reshape((3,1))
        self.currentForce = self.equivalentForceVector

        if self.qpForceRegulating:
            f_qp = qpContact.getContactForce()
            forceError = self.selectionMatrix.dot(self.equivalentForceVector - f_qp)
        else:
            forceError = self.selectionMatrix.dot(self.equivalentForceVector - self.desiredForce)

        #print ("The desired force is: ", self.desiredForce.T)
        #print ("The current sensor force is: ", self.equivalentForceVector.T)
        #print ("The force error is: ", forceError.T)



        self.error = forceError

        #
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        constant = newJacobian_dot.dot(dq) + self.Kf * forceError + self.Ki*self.forceErrorIntegral

        self.forceErrorIntegral = self.forceErrorIntegral + self.Ki*forceError*self.robot.world.dt

        Q = newJacobian_linear.T.dot(newJacobian_linear)
        Q_size = Q.shape[0]
        Q_new  = np.zeros((Q_size + self.robot.ndofs, Q_size + self.robot.ndofs))
        Q_new[:Q_size, :Q_size] = Q
        Q = Q_new

        P = 2 * constant.T.dot(newJacobian_linear)
        zero_vector = np.zeros((1, self.robot.ndofs))
        P = np.concatenate((P, zero_vector), axis=1)

        C = constant.T.dot(constant)

        if(useContactVariables):

            
            QP_original_weight = 1.0

            QP_size = 2*self.robot.ndofs
            contact_size = qpContact.Nc
            Q_new  = np.zeros((QP_size + contact_size, QP_size + contact_size))
            Q_new[:QP_size, :QP_size] = Q*QP_original_weight # write the old info
            
            P_new = np.zeros((1, QP_size + contact_size))
            P_new[0, :QP_size] = P*QP_original_weight

            if self.qpForceRegulating:
                weights = 10
                [force_Q, force_P, force_C] = self.calcQPRegulatingMatricies(qpContact)
                Q_new[2 * self.robot.ndofs:, 2 * self.robot.ndofs:] = force_Q*weights
                P_new[:, 2 * self.robot.ndofs:] = force_P*weights

        
            Q = Q_new
            P = P_new

        return [self.weight*Q, self.weight*P, self.weight*C]


    def motionCase(self, useContactVariables, qpContact):
        pass

    def calcMatricies(self, useContactVariables, qpContact):
        if self.velocityControl:
            return self.motionCase(useContactVariables, qpContact)
        else:
            return self.basicCase(useContactVariables, qpContact)

