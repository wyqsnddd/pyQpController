
import pydart2 as pydart
import numpy as np
from qpControllers import manipulatorQP
from manipulatorTasks import admittanceTask
from impactTask import impactEstimator
import executeACC

import logging
import time
import os

class manipulatorController:
    #def __init__(self, inputSkel, data, dt, logger=None):
    def __init__(self, inputSkel, data, dt):

        # if logger is None:
        #     raise Exception("Logger is not set")
        # else:
        #     self.logger = logger

        self.skel = inputSkel
        #self.targetPosition = inputTargetPosition
        self.enabled = True
        #self.accController = executeACC.jointAccController(inputSkel)

        #self.qp = manipulatorQP.manipulatorQP(self.skel, data, dt, logger)
        self.qp = manipulatorQP.manipulatorQP(self.skel, data, dt)

        self.joint_v_K_v = data["jointVelocityController"]["K_v"]
        self.joint_p_K_v = data["jointPositionController"]["K_v"]
        self.joint_p_K_p = data["jointPositionController"]["K_p"]
        self.box_bottom_friction_coeff = data["simulationWorldParameters"]["box_bottom_friction_coeff"]
        self.box_coarse_friction_coeff = data["simulationWorldParameters"]["box_coarse_surface_friction_coeff"]


        logger = logging.getLogger(__name__)
        logger.debug('manipulator controller created')
        self.errorZero = []
        self.time = []

        self.q_his = []
        self.dq_his = []
        self.acc_his = []
        self.tau_his = []
        
        self.sol_q_his = []
        self.sol_dq_his = []
        self.sol_acc_his = []
        self.sol_tau_his = []

        self.robot_c = []
        self.tau_last = np.zeros((self.skel.ndofs, 1))
        self.tau_last_two = np.zeros((self.skel.ndofs, 1))
        self.jointAcc_last = np.zeros((self.skel.ndofs, 1))
        self.solution = np.zeros((self.skel.ndofs, 1))
        self.constraintForceAware = data["qpController"]["constraintForceAware"]

        self.impactEstimatorEnabled = data["impactEstimator"]["enabled"]
        if(self.impactEstimatorEnabled):
            bodyNodeLink = data["impactEstimator"]["bodyLinkNumber"]
            self.impactEstimator = impactEstimator.impactEstimator(self.skel, bodyNodeLink)

        #self.dq_last = np.zeros((self.skel.ndofs, 1))
    # def updateTarget(self, inputTargetPosition):
    #     self.targetPosition = inputTargetPosition
    #     # update the reference point of each task:


    def solveQP(self):

        return self.qp.solve()

    def gravityCompensationTau(self):
        tau = np.zeros(self.skel.num_dofs())
        if not self.enabled:
            return tau

        for body in self.skel.bodynodes:
            m = body.mass()  # Or, simply body.m
            if (len(body.dependent_dofs) is not 0):
                J = body.linear_jacobian(body.local_com())
                # Each time we calculate all the torque that may affect the mass.
                tau += J.transpose().dot(-(m * self.skel.world.g))
        return tau

    def jointPositionControl(self, jointAcc):
        dt = self.skel.world.dt

        tau = 2*self.tau_last - self.tau_last_two + self.joint_p_K_p * jointAcc *dt*dt + self.joint_p_K_v*(jointAcc - self.jointAcc_last)*dt

        self.jointAcc_last = jointAcc

        self.tau_last_two = self.tau_last
        self.tau_last = tau

        tau = tau.flatten()

        #tau = tau + self.gravityCompensationTau()

        logger = logging.getLogger(__name__)
        logger.debug('The generated tau is %s ', tau)

        return tau

    def jointVelocityControl(self, jointAcc):

        #jointAccError = (np.reshape(self.skel.ddq, (self.skel.ndofs,1)) - jointAcc)
        #jointAccError = self.skel.ddq - jointAcc.reshape((1, self.skel.ndofs))

        #tau = self.tau_last + self.joint_v_K_v * (jointAccError) * self.skel.world.dt

        tau = self.tau_last + self.joint_v_K_v * (jointAcc) * self.skel.world.dt

        self.tau_last = tau
        #self.tau_last = (self.joint_v_K_v *self.skel.dq).reshape((self.skel.ndofs, 1))

        tau = tau.flatten()

        #tau = tau + self.gravityCompensationTau()
        tau = tau

        logger = logging.getLogger(__name__)
        logger.debug('The generated tau is %s ', tau)
        return tau

    def jointPDControl(self, jointAcc):

        dt = self.skel.world.dt
        dq = np.reshape(self.skel.dq, (self.skel.ndofs,1))
        q = np.reshape(self.skel.q, (self.skel.ndofs,1))

        predict_dq = dq + dt * jointAcc

        predict_q = q + dt * predict_dq

        positionError = q - predict_q

        velocityError = dq - predict_dq

        tau = - self.joint_p_K_v * velocityError - self.joint_p_K_p * positionError

        tau = tau.flatten()

        return tau



    def jointAccToTau(self, jointAcc):
        """!@brief
        Calculates the torque corresponding to the solved acceleration.
        @param jointAcc The resolved acceleration from the QP.
        """

        dt = self.skel.world.dt
        dq = np.reshape(self.skel.dq, (self.skel.ndofs, 1))
        q = np.reshape(self.skel.q, (self.skel.ndofs, 1))

        predict_dq = dq + dt * jointAcc

        self.sol_dq_his.append(predict_dq)

        predict_q = q + dt * predict_dq

        self.sol_q_his.append(predict_q)

        positionError = q - predict_q

        velocityError = dq - predict_dq

        # Use the end-effector Jacobian
        J = self.skel.bodynodes[-1].world_jacobian()

        # It seems that we can work with this equality if there is no forces
        # tau = self.skel.M.dot(jointAcc) + (self.skel.coriolis_and_gravity_forces()).reshape((self.skel.ndofs, 1)) - J.transpose().dot(self.skel.constraint_forces().reshape((self.skel.ndofs,1)))
        if(self.constraintForceAware):
            tau = self.skel.M.dot(jointAcc - self.joint_p_K_v * velocityError - self.joint_p_K_p * positionError) + (self.skel.coriolis_and_gravity_forces()).reshape((self.skel.ndofs, 1)) - self.skel.constraint_forces().reshape((self.skel.ndofs,1))
        else:
            tau = self.skel.M.dot(jointAcc - self.joint_p_K_v * velocityError - self.joint_p_K_p * positionError) + (self.skel.coriolis_and_gravity_forces()).reshape(
            (self.skel.ndofs, 1))

        self.sol_tau_his.append(tau)
        #print "The generated tau is: ", '\n', tau

        #return np.reshape(np.asarray(tau), (1, self.skel.ndofs))

        tau = tau.flatten()

        logger = logging.getLogger(__name__)
        logger.debug('The generated tau is %s ', tau)
        self.tau_last = tau

        return tau
    def logData(self):
        time_now = time.strftime("%b_%d_%Y_%H-%M-%S", time.gmtime())

        log_file_name = os.path.join('./log/data', 'data_' + time_now )
        np.savez_compressed(log_file_name, error=self.errorZero, time=self.time, dq=self.dq_his, robot_c=self.robot_c, acc = self.acc_his, q = self.q_his, tau = self.tau_his, sol_q = self.sol_q_his, sol_dq = self.sol_dq_his, sol_acc = self.sol_acc_his, sol_tau = self.sol_tau_his)

        if(self.impactEstimatorEnabled):
            impac_log_file_name = os.path.join('./log/data', 'impact-data_' + time_now)
            np.savez_compressed(impac_log_file_name, time=self.impactEstimator.time, predict_F=self.impactEstimator.predictionLog.impulsiveForce, predict_delta_tau=self.impactEstimator.predictionLog.deltaTorque, predict_delta_dq = self.impactEstimator.predictionLog.deltaDq, predict_average_acc = self.impactEstimator.predictionLog.averageDdq,
                                actual_F = self.impactEstimator.actualLog.impulsiveForce, actual_delta_tau = self.impactEstimator.actualLog.deltaTorque, actual_delta_dq = self.impactEstimator.actualLog.deltaDq)



    #
    def compute(self):
        """!@brief
        Would be called automatically by "step()" function of the world object
        """
        self.errorZero.append(self.qp.obj.tasks[0].error)
        self.time.append(self.skel.world.t)
        self.q_his.append(self.skel.q)
        self.dq_his.append(self.skel.dq)
        self.acc_his.append(self.skel.ddq)




        self.robot_c.append(self.skel.coriolis_and_gravity_forces())

        if (self.impactEstimatorEnabled):
            self.impactEstimator.update()

        # if self.skel.constraint_forces().sum() != 0.0:
        #     print "Impact detected"
        #     self.logData()

        self.solution = self.solveQP()
        logger = logging.getLogger(__name__)
        logger.debug('The generated joint acc is: %s ', self.solution)

        #print "The generated joint acc is: ", '\n', solution
        self.sol_acc_his.append(self.solution)
        tau = self.jointAccToTau(self.solution)
        self.tau_his.append(self.tau_last)

        return tau
        #return self.jointPositionControl(solution)
        #return self.jointVelocityControl(self.solution)
        #return [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        #return self.jointPDControl(solution)


