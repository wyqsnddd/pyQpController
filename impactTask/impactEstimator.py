import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array

import logging

from collections import namedtuple

class impactEstimator:
    """!@brief
    It generates joint acceleration given a desired position.
    """
    def __init__(self, skel, bodyNodeIndex = -1):
        self.robot = skel
        self.bodyNodeIndex = bodyNodeIndex

        self.updateParameters()
        self.initializeLog()

    def initializeLog(self):

        self.predictionLog = namedtuple('log', ['impulsiveForce','deltaTorque','deltaDq', 'averageDdq'])
        self.time = []
        self.predictionLog.impulsiveForce = []
        self.predictionLog.deltaTorque = []
        self.predictionLog.deltaDq = []
        self.predictionLog.averageDdq = []

        self.actualLog = namedtuple('log', ['impulsiveForce','deltaTorque','deltaDq'])
        self.actualLog.impulsiveForce = []
        self.actualLog.deltaTorque = []
        self.actualLog.deltaDq = []
        self.average_ddq = np.zeros((self.robot.ndofs, 1))

    def saveLog(self, predictF, predictTau, predictDq, averageDdq, F, tau, dq):

        self.time.append(self.robot.world.t)
        self.predictionLog.impulsiveForce.append(predictF)
        self.predictionLog.deltaTorque.append(predictTau)
        self.predictionLog.deltaDq.append(predictDq)
        self.predictionLog.averageDdq.append(averageDdq)

        self.actualLog.impulsiveForce.append(F)
        self.actualLog.deltaTorque.append(tau)
        self.actualLog.deltaDq.append(dq)



    def updateParameters(self):

        self.J_last = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()

        self.dq_last = (self.robot.dq).reshape((self.robot.ndofs, 1))

        self.F_last = np.linalg.pinv(self.J_last.T).dot(self.robot.constraint_forces())

        self.constraint_force_last = self.robot.constraint_forces()

        self.M_last = self.robot.M

        # Note that Tau_last is updated after the computation of joint controller
        #self.tau_last = self.robot.controller.tau_last

        # (J M J^T)_inv

        M_inv = np.linalg.pinv(self.M_last)
        temp = self.J_last.dot(M_inv)
        self.impactConstant_last = np.linalg.pinv(temp.dot(self.J_last.T))

        # update the end-effector(contact point) velocity jump
        self.contactVelocity_last =  self.robot.bodynodes[self.bodyNodeIndex].com_linear_velocity()

    def impactDetected(self):

        if self.robot.constraint_forces().sum() != 0.0:
            return True
        else:
            return False

    def checkContactVelocityJump(self):
        #return self.robot.bodynodes[self.bodyNodeIndex].com_linear_velocity() - self.contactVelocity_last
        return self.robot.bodynodes[self.bodyNodeIndex].com_linear_velocity()


    def calcImpulsiveQuantities(self):
        # After the impact detection, the robot configuration has already changed, thus we need to use values from before.
        delta_v = self.checkContactVelocityJump()

        dt_inv = 1/self.robot.world.dt

        temp = self.impactConstant_last.dot(delta_v)

        impulsiveF =  dt_inv*temp

        delta_torque = self.J_last.T.dot(impulsiveF)

        delta_joint_v = (np.linalg.pinv(self.M_last).dot(self.J_last.T)).dot(temp)

        delta_joint_v_simple = np.linalg.pinv(self.J_last).dot(delta_v)

        average_ddq = dt_inv*delta_joint_v
        self.average_ddq =  average_ddq
        return [impulsiveF, delta_torque, delta_joint_v, average_ddq, delta_joint_v_simple]
    
    def readAverageDdq(self):
        return self.average_ddq

    def checkImpulsiveQuantities(self):

        newJacobian_linear = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()

        M_inv = np.linalg.pinv(self.robot.M)
        temp = newJacobian_linear.dot(M_inv)
        impactConstant = np.linalg.pinv(temp.dot(newJacobian_linear.T))

        test_impulsiveF = impactConstant.dot(self.checkContactVelocityJump())

        # force difference:
        #impulsiveF = np.linalg.pinv(newJacobian_linear.T).dot(self.robot.constraint_forces())

        delta_torque = (self.robot.constraint_forces() - self.constraint_force_last)
        impulsiveF = np.linalg.pinv(newJacobian_linear.T).dot(delta_torque)

        delta_joint_v = ((self.robot.dq).reshape((self.robot.ndofs, 1)) - self.dq_last)

        return [impulsiveF, delta_torque, delta_joint_v]

    def update(self):
        if self.impactDetected():
            [predicted_F, predicted_delta_tau, predicted_delta_dq, predicted_average_ddq, predicted_delta_dq_sim] = self.calcImpulsiveQuantities()
            [real_F, real_delta_tau, real_delta_dq] = self.checkImpulsiveQuantities()
            self.saveLog(predicted_F, predicted_delta_tau, predicted_delta_dq,  predicted_average_ddq, real_F, real_delta_tau, real_delta_dq)

            print "Impact detected."
            self.updateParameters()
            # print "Difference between the predicted impulsive F and the real impulsive F is: ",'\n', predicted_F - real_F
            # print "Difference between the predicted delta torque and the real delta torque is: ", '\n', predicted_delta_tau - real_delta_tau
            # print "Difference between the predicted dq and the real dq is: ", '\n', predicted_delta_dq - real_delta_dq
            # print "Prediction finished. "
        else:
            # update the parameters as usual
            self.updateParameters()
