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
from qpControllers import manipulatorQP
from manipulatorTasks import contactAdmittanceTask, admittanceTask, positionTask, orientationTask
#from manipulatorController import  manipulatorController, jointVelocityJumpEstimator
from controllers import  executeACC, manipulatorController
from impactTask import impactEstimator, jointVelocityJumpEstimator

import logging
import time
import os

class manipulatorImpactController(manipulatorController.manipulatorController):
    #def __init__(self, inputSkel, data, dt, logger=None):
    def __init__(self, inputSkel, data, dt):
        manipulatorController.manipulatorController.__init__(self, inputSkel, data, dt)


        self.switchedTasks = False

    def impactDetected(self):

        if self.skel.constraint_forces().sum() != 0.0:
            return True
        else:
            return False

    def addAdmittanceTask(self):
        logger = logging.getLogger(__name__)

        test_desiredForce = self.qp.data["qpController"]["admittanceTask"]["desiredForce"]
        test_weight = self.qp.data["qpController"]["admittanceTask"]["taskWeight"]
        qpRegulating = self.qp.data["qpController"]["admittanceTask"]["qpForceRegulating"]

        Kf = self.qp.data["qpController"]["admittanceTask"]["Kf"]
        Ki = self.qp.data["qpController"]["admittanceTask"]["Ki"]
        linkIndex = self.qp.data["qpController"]["admittanceTask"]["bodyLinkNumber"]

        test_desiredForce = np.asarray(test_desiredForce).reshape((3, 1))

        test_selectionVector = self.qp.data["qpController"]["admittanceTask"]["axis_selection"]
        test_selectionVector = np.asarray(test_selectionVector).reshape((3, 1))

        logger.info("desired force is: %s, controller gains are %d and %d. End-effector is link: %d ",
                    test_desiredForce, Ki, Kf, linkIndex)
        newAdmittanceTask = admittanceTask.admittanceTask(self.skel, test_desiredForce, test_weight,
                                                          test_selectionVector, Kf=Kf, Ki=Ki,
                                                          bodyNodeIndex=linkIndex,
                                                          qpForceRegulating=qpRegulating)
        self.qp.obj.addTask(newAdmittanceTask)

        logger.info("initialized admittance task ")
        print ("The admittance task selection matrix is: ", '\n', self.qp.obj.tasks[0].selectionMatrix)

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
        print ("The admittance task selection matrix is: ", '\n', self.qp.obj.tasks[0].selectionMatrix)

    def addPositionTask(self):
        logger = logging.getLogger(__name__)


        test_weight = self.qp.data["qpController"]["positionTask"]["taskWeight"]

        Kp = self.qp.data["qpController"]["positionTask"]["Kp"]
        Kd = self.qp.data["qpController"]["positionTask"]["Kd"]
        linkIndex = self.qp.data["qpController"]["positionTask"]["bodyLinkNumber"]

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

    def addOrientationTask(self):
        logger = logging.getLogger(__name__)

        test_stableStill = self.qp.data["qpController"]["orientationTask"]["stayStill"]
        Kp = self.qp.data["qpController"]["orientationTask"]["Kp"]
        Kd = self.qp.data["qpController"]["orientationTask"]["Kd"]
        linkIndex = self.qp.data["qpController"]["orientationTask"]["bodyLinkNumber"]

        if(test_stableStill):
            transform = self.skel.bodynodes[linkIndex].world_transform()
            rotation = transform[:3, :3]
            test_desiredOrientation = pydart.utils.transformations.quaternion_from_matrix(rotation)
        else:
            test_desiredOrientation = self.qp.data["qpController"]["orientationTask"]["setPoint"]
            
        test_weight = self.qp.data["qpController"]["orientationTask"]["taskWeight"]
        scalar_weight = self.qp.data["qpController"]["orientationTask"]["quaternion_scalar_weight"]

                
        test_selectionVector = self.qp.data["qpController"]["orientationTask"]["axis_selection"]
        test_selectionVector = np.asarray(test_selectionVector).reshape((3, 1))
        
        newOrientationTask = orientationTask.orientationTask(self.skel, test_desiredOrientation, test_weight, scalar_weight, test_selectionVector, Kd, Kp, linkIndex)


        self.qp.obj.addTask(newOrientationTask)
        logger.info("initialized orientation task ")

        
    def compute(self):
        """!@brief
        Would be called automatically by "step()" function of the world object
        """


        jacobian = self.skel.bodynodes[-1].linear_jacobian()


        self.errorZero.append(self.qp.obj.tasks[0].error)
        self.time.append(self.skel.world.t)
        self.ee_v_his.append(jacobian.dot(self.skel.dq))
        self.ee_f_his.append(np.linalg.pinv(jacobian.T).dot(self.skel.constraint_forces()))
        self.dq_his.append(self.skel.dq)
        self.robot_c.append(self.skel.coriolis_and_gravity_forces())
        self.q_his.append(self.skel.q)
        self.acc_his.append(self.skel.ddq)

        self.robot_c.append(self.skel.coriolis_and_gravity_forces())

        if (self.impactEstimatorEnabled):
            self.impactEstimator.update()


        if self.impactDetected() & (self.switchedTasks == False):
            print ("Impact detected")
            self.logData()

            # Notify the QP that we are in the contact mode
            self.qp.setContactStatus()

            # turn off the x axis control
            #self.qp.obj.tasks[0].selectionMatrix[0, 0] = 0.0

            # remove all the tasks:
            n_tasks =  len(self.qp.obj.tasks)
            for ii in range(0, n_tasks):
                self.qp.obj.tasks.remove(self.qp.obj.tasks[n_tasks - ii - 1])


            self.switchedTasks = True

            logger = logging.getLogger(__name__)
            logger.info("translational velocity task is removed")
            transform = self.skel.bodynodes[-1].world_transform()
            translation = transform[[0, 1, 2], 3].reshape((3, 1))

            #self.addContactAdmittanceTask(ee_position=translation)

            self.addAdmittanceTask()

            self.addPositionTask()
            if(self.qp.data["qpController"]["orientationTask"]["enabled"]):
                self.addOrientationTask()


        [self.sol_ddq, self.sol_delta_dq, self.sol_weights ]= self.solveQP()
        self.solution = self.sol_ddq

        logger = logging.getLogger(__name__)
        logger.debug('The generated joint acc is: %s ', self.solution)



        #print "The generated joint acc is: ", '\n', solution
        if (self.jointVelocityJumpEstimatorEnabled):
            self.jointVelocityJumpEstimator.update(self.sol_ddq, self.sol_delta_dq)

        self.sol_acc_his.append(self.solution)
        tau = self.jointAccToTau(self.solution)
        self.tau_his.append(self.tau_last)

        self.sol_lambda_his.append(self.sol_weights)
        self.f_QP_his.append(np.reshape(self.qp.contact.getContactGenerationMatrix().dot(self.sol_weights), (3, 1)))

        return tau

