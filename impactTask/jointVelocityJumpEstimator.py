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


    def __init__(self, skel, resLower = 0.4, resUpper = 0.6, bodyNodeIndex = -1, tauUpper=None, tauLower=None):
        self.robot = skel
        self.bodyNodeIndex = bodyNodeIndex

        self.updateParameters()
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


        if tauUpper is None:
            self.tauUpper = self.robot.tau_upper.reshape((self.robot.num_dofs(), 1))
        else:
            if tauUpper.shape == (self.robot.num_dofs(),1):
                self.tauUpper = tauUpper
            else:
                raise Exception("Upper torque limit size does not match")

        if tauLower is None:
            self.tauLower = self.robot.tau_lower.reshape((self.robot.num_dofs(), 1))
        else:
            if tauLower.shape == (self.robot.num_dofs(), 1):
                self.tauLower = tauLower
            else:
                raise Exception("Lower torque limit size does not match")
        
        self.dt = self.robot.world.dt

    def initializeLog(self):

        self.predictionLog = namedtuple('log', ['dq','ddq','tau','delta_dq_lower','delta_dq_upper', 'ddqUpperBoundPosition', 'ddqLowerBoundPosition','ddqUpperBoundVelocity', 'ddqLowerBoundVelocity', 'real_ddqUpperBoundPosition', 'real_ddqLowerBoundPosition','real_ddqUpperBoundVelocity', 'real_ddqLowerBoundVelocity', 'real_ddqUpperBoundTau', 'real_ddqLowerBoundTau', 'predict_ddqUpperBoundTau', 'predict_ddqLowerBoundTau','predict_tauUpper', 'predict_tauLower', 'predict_impulseTorque', 'impulseTorque'])
        
        self.time = []
        self.predictionLog.ddq = []
        self.predictionLog.dq = []
        self.predictionLog.tau = []

        self.predictionLog.deltaDqLower = []
        self.predictionLog.deltaDqUpper = []
        
        self.predictionLog.ddqUpperBoundPosition = []
        self.predictionLog.ddqLowerBoundPosition = []
        self.predictionLog.real_ddqUpperBoundPosition = []
        self.predictionLog.real_ddqLowerBoundPosition = []

        self.predictionLog.ddqUpperBoundVelocity = []
        self.predictionLog.ddqLowerBoundVelocity = []
        self.predictionLog.real_ddqUpperBoundVelocity = []
        self.predictionLog.real_ddqLowerBoundVelocity = []

        self.predictionLog.real_ddqUpperBoundTau = []
        self.predictionLog.real_ddqLowerBoundTau = []
        
        self.predictionLog.predict_ddqUpperBoundTau = []
        self.predictionLog.predict_ddqLowerBoundTau = []

        self.predictionLog.predict_tauUpper = []
        self.predictionLog.predict_tauLower = []
        
        self.predictionLog.predict_impulseTau = []
        self.predictionLog.impulseTau = []

    def saveLog(self, dq, ddq, tau,
                predictDeltaDqLower, predictDeltaDqUpper,
                ddqLowerPosition, ddqUpperPosition,
                real_ddqLowerPosition, real_ddqUpperPosition,
                ddqLowerVelocity, ddqUpperVelocity,
                real_ddqLowerVelocity, real_ddqUpperVelocity,
                real_ddqLowerTau, real_ddqUpperTau,
                predict_ddqLowerTau, predict_ddqUpperTau,
                predict_tauLower, predict_tauUpper,
                predict_impulseTau, impulseTau
    ):


        self.time.append(self.robot.world.t)
        self.predictionLog.dq.append(dq)
        self.predictionLog.ddq.append(ddq)
        self.predictionLog.tau.append(tau)

        self.predictionLog.deltaDqLower.append(predictDeltaDqLower)
        self.predictionLog.deltaDqUpper.append(predictDeltaDqUpper)

        self.predictionLog.ddqUpperBoundPosition.append(ddqUpperPosition)
        self.predictionLog.ddqLowerBoundPosition.append(ddqLowerPosition)
        self.predictionLog.ddqUpperBoundVelocity.append(ddqUpperVelocity)
        self.predictionLog.ddqLowerBoundVelocity.append(ddqLowerVelocity)

        self.predictionLog.real_ddqUpperBoundPosition.append(real_ddqUpperPosition)
        self.predictionLog.real_ddqLowerBoundPosition.append(real_ddqLowerPosition)
        self.predictionLog.real_ddqUpperBoundVelocity.append(real_ddqUpperVelocity)
        self.predictionLog.real_ddqLowerBoundVelocity.append(real_ddqLowerVelocity)
        self.predictionLog.real_ddqUpperBoundTau.append(real_ddqUpperTau)
        self.predictionLog.real_ddqLowerBoundTau.append(real_ddqLowerTau)

        self.predictionLog.predict_ddqUpperBoundTau.append(predict_ddqUpperTau)
        self.predictionLog.predict_ddqLowerBoundTau.append(predict_ddqLowerTau)

        self.predictionLog.predict_tauUpper.append(predict_tauUpper)
        self.predictionLog.predict_tauLower.append(predict_tauLower)
        
        self.predictionLog.predict_impulseTau.append(predict_impulseTau)
        self.predictionLog.impulseTau.append(impulseTau)


    def updateParameters(self):
        self.dq_last = (self.robot.dq).reshape((self.robot.ndofs, 1))
        self.q_last = (self.robot.q).reshape((self.robot.ndofs, 1))
        self.M_inv_last = np.linalg.pinv(self.robot.M)
        self.N_last = (self.robot.coriolis_and_gravity_forces()).reshape((self.robot.ndofs, 1))
        
    def calcImpulsiveQuantities(self, sol_ddq):

        N = (self.robot.coriolis_and_gravity_forces()).reshape((self.robot.ndofs, 1))

        sol_tau = self.robot.M.dot(sol_ddq) + N

        M_inv = np.linalg.pinv(self.robot.M)

        jacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()

        temp_1 = M_inv.dot(jacobian.T)

        temp_2 = np.linalg.pinv(jacobian.dot(temp_1))

        constant = temp_1.dot(temp_2)

        dq =  (self.robot.dq).reshape((self.robot.ndofs, 1))
        q = (self.robot.q).reshape((self.robot.ndofs, 1))


        baseOne = constant.dot(jacobian.dot( dq + self.dt*sol_ddq   ))

        predicted_delta_dq_upper = (- self.resUpper - 1)*baseOne
        predicted_delta_dq_lower = (- self.resLower - 1)*baseOne

        ddq = (self.robot.ddq).reshape((self.robot.ndofs, 1))
        position = (self.robot.q).reshape((self.robot.ndofs, 1))


        ddq_upper_bound_position = (self.positionUpper - position + ddq*(self.dt*self.dt) + predicted_delta_dq_upper*self.dt)/(self.dt*self.dt)
        ddq_lower_bound_position = (self.positionLower - position + ddq * (self.dt*self.dt) + predicted_delta_dq_lower * self.dt) / (
        self.dt*self.dt)

        ddq_upper_bound_Velocity = -(self.velocityLower - dq - ddq * self.dt - predicted_delta_dq_upper) / self.dt
        ddq_lower_bound_Velocity = -(self.velocityUpper - dq - ddq*self.dt - predicted_delta_dq_lower)/self.dt


        real_ddq_upper_bound_position =  (self.positionUpper - q - dq*self.dt)/(self.dt*self.dt)
        real_ddq_lower_bound_position = (self.positionLower - q - dq*self.dt)/(self.dt*self.dt)
        
        real_ddq_upper_bound_Velocity =  (self.velocityUpper - dq) / self.dt
        real_ddq_lower_bound_Velocity =  (self.velocityLower - dq) / self.dt



        real_ddq_lower_bound_tau = M_inv.dot(self.tauLower - N)
        real_ddq_upper_bound_tau = M_inv.dot(self.tauUpper - N)

        c_temp = (self.robot.coriolis_and_gravity_forces()).reshape((self.robot.ndofs, 1))
        # Reset the states:
        temp_v_upper = (predicted_delta_dq_upper + dq).flatten()
        #temp_v_upper = (predicted_delta_dq_upper).flatten()

        self.robot.set_velocities(temp_v_upper)
        N_upper = (self.robot.coriolis_and_gravity_forces()).reshape((self.robot.ndofs, 1))
        predict_ddq_upper_bound_tau = self.M_inv_last.dot(self.tauUpper - N_upper)

        predict_tau_upper = self.robot.M.dot(sol_ddq) + N_upper
        
        temp_v_lower = (predicted_delta_dq_lower + dq).flatten()
        #temp_v_lower = (predicted_delta_dq_lower).flatten()

        self.robot.set_velocities(temp_v_lower)
        N_lower = (self.robot.coriolis_and_gravity_forces()).reshape((self.robot.ndofs, 1))
        predict_ddq_lower_bound_tau = self.M_inv_last.dot(self.tauLower - N_lower)

        predict_tau_lower = self.robot.M.dot(sol_ddq) + N_lower
        
        predict_impulse_tau = self.robot.M.dot(predicted_delta_dq_upper)/self.dt
        impulse_tau = self.robot.constraint_forces()

        self.robot.set_velocities(dq.flatten())
        
        return [dq, sol_ddq, sol_tau,
                predicted_delta_dq_lower, predicted_delta_dq_upper,
                ddq_lower_bound_position, ddq_upper_bound_position,
                real_ddq_lower_bound_position, real_ddq_upper_bound_position,
                ddq_lower_bound_Velocity, ddq_upper_bound_Velocity,
                real_ddq_lower_bound_Velocity, real_ddq_upper_bound_Velocity,
                real_ddq_lower_bound_tau, real_ddq_upper_bound_tau,
                predict_ddq_lower_bound_tau, predict_ddq_upper_bound_tau,
                predict_tau_lower, predict_tau_upper,
                predict_impulse_tau, impulse_tau
                ]



    def update(self, sol_ddq):

        [dq, sol_ddq, sol_tau,
         predicted_delta_dq_lower, predicted_delta_dq_upper,
         ddq_lower_bound_position, ddq_upper_bound_position,
         real_ddq_lower_bound_position, real_ddq_upper_bound_position,
         ddq_lower_bound_Velocity, ddq_upper_bound_Velocity,
         real_ddq_lower_bound_Velocity, real_ddq_upper_bound_Velocity,
         real_ddq_lower_bound_tau, real_ddq_upper_bound_tau,
         predict_ddq_lower_bound_tau, predict_ddq_upper_bound_tau,
         predict_tau_lower, predict_tau_upper,
         predict_impulse_tau, impulse_tau
         ] = self.calcImpulsiveQuantities(sol_ddq)
        



        self.saveLog(dq, sol_ddq, sol_tau,
                     predicted_delta_dq_lower, predicted_delta_dq_upper,
                     ddq_lower_bound_position, ddq_upper_bound_position,
                     real_ddq_lower_bound_position, real_ddq_upper_bound_position,
                     ddq_lower_bound_Velocity, ddq_upper_bound_Velocity,
                     real_ddq_lower_bound_Velocity, real_ddq_upper_bound_Velocity,
                     real_ddq_lower_bound_tau, real_ddq_upper_bound_tau,
                     predict_ddq_lower_bound_tau, predict_ddq_upper_bound_tau,
                     predict_tau_lower, predict_tau_upper,
                     predict_impulse_tau, impulse_tau)
        

        self.updateParameters()
