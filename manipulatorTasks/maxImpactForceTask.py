import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array

import logging


class maxImpactForceTask:
    """!@brief
    @class maxImpactForceTask
    Maximize the impact force along an given unit direction 
    """
    def __init__(self, skel, desiredDirection, taskWeight, bodyNodeIndex=None):

        logger = logging.getLogger(__name__)
        self.robot = skel

        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex

        if desiredDirection is None:
            raise Exception("Desired direction is not set")
        else:
            vecNorm = np.linalg.norm(desiredDirection)
            np.reshape( desiredDirection, (3,1))
            desiredDirection = desiredDirection/vecNorm
            self.desiredDirection = np.asarray(desiredDirection).reshape((3, 1))
            self.projectionMatrix = self.desiredDirection.dot(self.desiredDirection.transpose())

        if taskWeight is None:
            raise Exception("Task weight is not set")
        else:
            self.taskWeight = taskWeight
            
        self.error = np.zeros((3, 1))
        #logger.warning("Kd is: %d", self.Kd)
    def update(self):
        pass
    def calcMatricies(self, useContactVariables, qpContact):
        
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        jacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian = self.projectionMatrix.dot(jacobian)
        jacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.projectionMatrix.dot(jacobian_dot)

        v_n = self.projectionMatrix.dot(jacobian.dot(dq))
        
        M_inv = np.linalg.pinv(self.robot.M)
        Meq = np.linalg.pinv(jacobian.dot(M_inv).dot(jacobian.transpose()))


        
        M_temp = Meq.transpose().dot(Meq)

        P = (v_n.transpose().dot(M_temp)).dot(jacobian)
        
        zero_vector = np.zeros((1, self.robot.ndofs))
        P = np.concatenate((P, zero_vector), axis=1)

        C = 1
        Q = np.zeros((2*self.robot.ndofs, 2*self.robot.ndofs))

        if(useContactVariables):
            QP_size = 2*self.robot.ndofs
            contact_size = qpContact.Nc
            Q_new  = np.zeros((QP_size + contact_size, QP_size + contact_size))
            #Q_new[:QP_size, :QP_size] = Q # write the old info
            Q = Q_new

            P_new = np.zeros((1, QP_size + contact_size))
            P_new[0, :QP_size] = P
            P = P_new

        return [Q, - self.taskWeight*P, - self.taskWeight*C]
