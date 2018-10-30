
import pydart2 as pydart
import numpy as np
from qpControllers import manipulatorQP
from manipulatorTasks import contactAdmittanceTask, admittanceTask, positionTask
from manipulatorController import  manipulatorController
import executeACC

from impactTask import impactEstimator

import logging
import time
import os

class manipulatorImpactController(manipulatorController):
    #def __init__(self, inputSkel, data, dt, logger=None):
    def __init__(self, inputSkel, data, dt):
        manipulatorController.__init__(self, inputSkel, data, dt)

        # logger = logging.getLogger(__name__)
        # logger.debug('manipulator controller created')
        # self.errorZero = []
        # self.time = []
        # self.dq = []
        # self.robot_c = []

        #bodyNodeLink = data["impactEstimator"]["bodyLinkNumber"]
        #self.impactEstimator = impactEstimator.impactEstimator(self.skel, bodyNodeLink)

        self.switchedTasks = False
    # def updateTarget(self, inputTargetPosition):
    #     self.targetPosition = inputTargetPosition
    #     # update the reference point of each task:

    def logData(self):
        time_now = time.strftime("%b_%d_%Y_%H-%M-%S", time.gmtime())

        log_file_name = os.path.join('./log/data', 'data_' + time_now )

        np.savez_compressed(log_file_name, error=self.errorZero, time=self.time, dq=self.dq_his, robot_c=self.robot_c,
                            acc=self.acc_his, q=self.q_his, tau=self.tau_his, sol_q=self.sol_q_his,
                            sol_dq=self.sol_dq_his, sol_acc=self.sol_acc_his, sol_tau=self.sol_tau_his)

        if(self.impactEstimatorEnabled):
            impac_log_file_name = os.path.join('./log/data', 'impact-data_' + time_now)
            np.savez_compressed(impac_log_file_name, time=self.impactEstimator.time, predict_F=self.impactEstimator.predictionLog.impulsiveForce, predict_delta_tau=self.impactEstimator.predictionLog.deltaTorque, predict_delta_dq = self.impactEstimator.predictionLog.deltaDq,
                                actual_F = self.impactEstimator.actualLog.impulsiveForce, actual_delta_tau = self.impactEstimator.actualLog.deltaTorque, actual_delta_dq = self.impactEstimator.actualLog.deltaDq)



    def impactDetected(self):

        if self.skel.constraint_forces().sum() != 0.0:
            return True
        else:
            return False

    def addAdmittanceTask(self):
        logger = logging.getLogger(__name__)

        test_desiredForce = self.qp.data["qpController"]["admittanceTask"]["desiredForce"]
        test_weight = self.qp.data["qpController"]["admittanceTask"]["taskWeight"]

        Kf = self.qp.data["qpController"]["admittanceTask"]["Kf"]
        Ki = self.qp.data["qpController"]["admittanceTask"]["Ki"]
        linkIndex = self.qp.data["qpController"]["admittanceTask"]["bodyLinkNumber"]

        # test_desiredPosition = listToArray.listToArray(test_desiredPosition)
        test_desiredForce = np.asarray(test_desiredForce).reshape((3, 1))

        test_selectionVector = self.qp.data["qpController"]["admittanceTask"]["axis_selection"]
        test_selectionVector = np.asarray(test_selectionVector).reshape((3, 1))

        logger.info("desired force is: %s, controller gains are %d and %d. End-effector is link: %d ",
                    test_desiredForce, Ki, Kf, linkIndex)
        # test_vector = np.zeros(5)
        # initialize the tasks:
        newAdmittanceTask = admittanceTask.admittanceTask(self.skel, test_desiredForce, test_weight,
                                                          test_selectionVector, Kf=Kf, Ki=Ki,
                                                          bodyNodeIndex=linkIndex)
        self.qp.obj.addTask(newAdmittanceTask)

        logger.info("initialized admittance task ")
        print "The admittance task selection matrix is: ", '\n', self.qp.obj.tasks[0].selectionMatrix

    def addContactAdmittanceTask(self, ee_position):
        logger = logging.getLogger(__name__)

        test_desiredForce = self.qp.data["qpController"]["contactAdmittanceTask"]["desiredForce"]
        test_weight = self.qp.data["qpController"]["contactAdmittanceTask"]["taskWeight"]

        Kp = self.qp.data["qpController"]["contactAdmittanceTask"]["Kp"]
        Kd = self.qp.data["qpController"]["contactAdmittanceTask"]["Kd"]
        Kf = self.qp.data["qpController"]["contactAdmittanceTask"]["Kf"]
        Ki = self.qp.data["qpController"]["contactAdmittanceTask"]["Ki"]

        linkIndex = self.qp.data["qpController"]["contactAdmittanceTask"]["bodyLinkNumber"]

        # test_desiredPosition = listToArray.listToArray(test_desiredPosition)
        test_desiredForce = np.asarray(test_desiredForce).reshape((3, 1))

        test_selectionVector = self.qp.data["qpController"]["contactAdmittanceTask"]["axis_selection"]
        test_selectionVector = np.asarray(test_selectionVector).reshape((3, 1))

        logger.info("desired force is: %s, controller gains are %d and %d. End-effector is link: %d ",
                    test_desiredForce, Ki, Kf, linkIndex)
        # test_vector = np.zeros(5)
        # initialize the tasks:

        # here we use the current robot
        test_desiredPosition = ee_position

        newAdmittanceTask = contactAdmittanceTask.contactAdmittanceTask(self.skel, test_desiredForce, test_weight, test_desiredPosition,
                                                          Kd=Kd, Kp=Kp, Kf=Kf, Ki=Ki,
                                                          selectionVector=test_selectionVector, bodyNodeIndex=linkIndex)
        self.qp.obj.addTask(newAdmittanceTask)

        logger.info("initialized admittance task ")
        print "The admittance task selection matrix is: ", '\n', self.qp.obj.tasks[0].selectionMatrix

    def addPositionTask(self):
        logger = logging.getLogger(__name__)


        test_weight = self.qp.data["qpController"]["positionTask"]["taskWeight"]

        Kp = self.qp.data["qpController"]["positionTask"]["Kp"]
        Kd = self.qp.data["qpController"]["positionTask"]["Kd"]
        linkIndex = self.qp.data["qpController"]["positionTask"]["bodyLinkNumber"]

        # test_desiredPosition = listToArray.listToArray(test_desiredPosition)
        #test_desiredPosition = self.qp.data["qpController"]["positionTask"]["setPoint"]
        #test_desiredPosition = np.asarray(test_desiredPosition).reshape((3, 1))


        # test_vector = np.zeros(5)
        # initialize the tasks:

        test_selectionVector = self.qp.data["qpController"]["positionTask"]["axis_selection"]
        test_selectionVector = np.asarray(test_selectionVector).reshape((3, 1))

        transform = self.skel.bodynodes[-1].world_transform()
        robot_ee_translation = transform[:3, 3]
        robot_ee_translation = np.asarray(robot_ee_translation).reshape((3, 1))
        logger.info("desired position is: %s, controller gains are %d and %d. End-effector is link: %d ",
                    robot_ee_translation, Kd, Kp, linkIndex)

        newPositionTask = positionTask.positionTask(self.skel, robot_ee_translation, test_weight, test_selectionVector,
                                                    Kd, Kp, linkIndex)
        self.qp.obj.addTask(newPositionTask)
        logger.info("initialized position task ")

    def compute(self):
        """!@brief
        Would be called automatically by "step()" function of the world object
        """
        self.errorZero.append(self.qp.obj.tasks[0].error)
        self.time.append(self.skel.world.t)
        self.dq_his.append(self.skel.dq)
        self.robot_c.append(self.skel.coriolis_and_gravity_forces())
        self.q_his.append(self.skel.q)
        self.acc_his.append(self.skel.ddq)

        if (self.impactEstimatorEnabled):
            self.impactEstimator.update()


        if self.impactDetected() & (self.switchedTasks == False):
            print "Impact detected"
            self.logData()
            # turn off the x axis control
            #self.qp.obj.tasks[0].selectionMatrix[0, 0] = 0.0
            self.qp.obj.tasks.remove(self.qp.obj.tasks[0])
            self.switchedTasks = True

            logger = logging.getLogger(__name__)
            logger.info("translational velocity task is removed")
            #print "The velocity task selection matrix is: ",'\n', self.qp.obj.tasks[0].selectionMatrix
            #logger.info("The amount of tasks is: %d", len(self.qp.obj.tasks))

            # self.addAdmittanceTask()
            transform = self.skel.bodynodes[-1].world_transform()
            translation = transform[[0, 1, 2], 3].reshape((3, 1))
            self.addContactAdmittanceTask(ee_position=translation)
            self.addPositionTask()


        self.solution = self.solveQP()
        logger = logging.getLogger(__name__)
        logger.debug('The generated joint acc is: %s ', self.solution)

        #print "The generated joint acc is: ", '\n', solution

        self.sol_acc_his.append(self.solution)
        tau = self.jointAccToTau(self.solution)
        self.tau_his.append(self.tau_last)
        return tau
        #return self.gravityCompensationTau()

        #return self.jointVelocityControl(solution)

        #return self.jointPositionControl(solution)


