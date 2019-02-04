import pydart2 as pydart

import _pydart2_api as papi

import numpy as np

from cvxopt import matrix, solvers

from manipulatorConstraints import jointPositionLimits, jointVelocityConstraints, jointAccelerationConstraints, torqueLimitConstraints, impactConstraints, impactBoundConstraints, impulseTorqueLimitConstraints, combinedTorqueConstraints, robustJointVelocityConstraints, robustJointPositionLimits

from qpControllers import qpObj

from manipulatorTasks import positionTask
from manipulatorTasks import orientationTask
from manipulatorTasks import translationVelocityTask
from manipulatorTasks import maxContactVelTask
from manipulatorTasks import maxImpactForceTask
from manipulatorTasks import trajectoryTask
from contact import qpContact, contactEqualityConstraint

#from qpsolvers import solve_qp

from cvxopt import matrix, spmatrix
from cvxopt.solvers import options, qp


from utils import listToArray
import logging


import json

class manipulatorQP:
    """!@brief
    QP for a manipulator, which includes basic constraints:
    (1) joint position, velocity and torque limits
    (2) tasks could be specified later on

    We solve for the joint accelerations
    """

    def __init__(self, skel, data, dt):

        self.robot = skel
        self.dof = self.robot.ndofs
        self.data = data

        self.initializeParameters(data, dt)
        showProgress = data["qpController"]["showProgress"]
        solvers.options['show_progress'] = showProgress

    def initializeParameters(self, data, dt):

        logger = logging.getLogger(__name__)

        #self.jointsUpper = np.reshape(self.robot.q_upper(), (self.dof,1))
        #self.jointsLower = np.reshape(self.robot.q_lower(), (self.dof,1))
        #dt = 0.01
        #dt = data["qpController"]["jointLimits"]["dt"]
        print "dt is:", dt

        self.contactAware = data["qpController"]["contact"]["enabled"]
        self.contactWeight = data["qpController"]["contact"]["weight"]

        self.contactStart = False
        self.contact = qpContact.qpContact(self.robot)
        #if(self.contactAware):
            # We do not use the equality, instead, we use the robot dynamics.
            #self.contactEqualityConstraint = contactEqualityConstraint.contactEqualityConstraint(self.robot, self.contact)

        logger.info("initialized QP contact ")

        self.impactRobust = data["qpController"]["impactRobust"]

        self.robustJointVelocityLimitConstraints = robustJointVelocityConstraints.robustJointVelocityLimitConstraints(self.robot, dt)
        self.robustJointLimitConstraints = robustJointPositionLimits.robustJointLimitConstraints(self.robot, dt)


        self.jointVelocityLimitConstraints = jointVelocityConstraints.jointVelocityLimitConstraints(self.robot, dt)
        self.jointLimitConstraints = jointPositionLimits.jointLimitConstraints(self.robot, dt)

        self.jointAccelerationLimitConstraints = jointAccelerationConstraints.jointAccelerationLimitConstraints(self.robot)

        # The joint velocity limtis is not given by skeleton
        # self.jointsVUpper =
        # self.jointsVLower =

        upper = data["qpController"]["torqueLimits"]["upper"]
        lower = data["qpController"]["torqueLimits"]["lower"]

        # if self.impactRobust:
        #     upper = data["qpController"]["torqueLimits"]["upper"]
        #     lower = data["qpController"]["torqueLimits"]["lower"]
        # else:
        #     upper = data["qpController"]["impulseTorqueLimits"]["upper"]
        #     lower = data["qpController"]["impulseTorqueLimits"]["lower"]


        self.torqueLower = np.reshape(np.asarray(lower), (self.dof, 1))
        self.torqueUpper = np.reshape(np.asarray(upper), (self.dof, 1))
        logger.info("torque upper limit is: %s ",  self.torqueUpper)
        logger.info("torque lower limit is: %s ", self.torqueLower)

        # print "torque upper limit is: ", self.torqueUpper
        # print "torque lower limit is: ", self.torqueLower

        # self.torqueLower = np.reshape(self.robot.tau_lower, (self.dof, 1))
        # self.torqueUpper = np.reshape(self.robot.tau_upper, (self.dof, 1))

        self.torqueLimitConstraints = torqueLimitConstraints.torqueLimitConstraints(self.robot, self.impactRobust, self.torqueUpper, self.torqueLower)

        impulseUpper = data["qpController"]["impulseTorqueLimits"]["upper"]
        impulseLower = data["qpController"]["impulseTorqueLimits"]["lower"]

        self.impulseTorqueLower = np.reshape(np.asarray(impulseLower), (self.dof, 1))
        self.impulseTorqueUpper = np.reshape(np.asarray(impulseUpper), (self.dof, 1))
        
        logger.info("impulse torque upper limit is: %s ",  self.impulseTorqueUpper)
        logger.info("impulse torque lower limit is: %s ", self.impulseTorqueLower)

        # print "torque upper limit is: ", self.torqueUpper
        # print "torque lower limit is: ", self.torqueLower

        # self.torqueLower = np.reshape(self.robot.tau_lower, (self.dof, 1))
        # self.torqueUpper = np.reshape(self.robot.tau_upper, (self.dof, 1))

        self.impulseTorqueLimitConstraints = impulseTorqueLimitConstraints.impulseTorqueLimitConstraints(self.robot, self.impulseTorqueUpper, self.impulseTorqueLower)
        #self.impulseTorqueLimitConstraints = combinedTorqueConstraints.combinedTorqueLimitConstraints(self.robot, self.impulseTorqueUpper, self.impulseTorqueLower)
        
        resCoe = data["simulationWorldParameters"]["palm_restitution_coeff"]
        self.impactConstraints = impactConstraints.impactConstraints(self.robot, resCoe)
        #self.impactBoundConstraints = impactBoundConstraints.impactBoundConstraints(self.robot, resCoe)

        logger.info("initialized constraints ")

        # initialize objective function:
        test_jointUnitWeight = data["qpController"]["jointUnitWeight"]
        test_deltaDqWeight = data["qpController"]["deltaDqUnitWeight"]
        test_contactWeight = data["qpController"]["contactUnitWeight"]

        self.obj = qpObj.qpObj(self.robot, test_jointUnitWeight, test_deltaDqWeight, test_contactWeight, self.contact)
        logger.info("initialized objective ")
        #print "initialized objective "

        #test_desiredPosition = np.array([0.1, 0.2, 0.3]).reshape((3, 1))
        test_positionTask = data["qpController"]["positionTask"]["enabled"]

        if(test_positionTask):
            test_desiredPosition = data["qpController"]["positionTask"]["setPoint"]
            test_weight = data["qpController"]["positionTask"]["taskWeight"]

            Kp = data["qpController"]["positionTask"]["Kp"]
            Kd = data["qpController"]["positionTask"]["Kd"]
            linkIndex = data["qpController"]["positionTask"]["bodyLinkNumber"]

            #test_desiredPosition = listToArray.listToArray(test_desiredPosition)
            test_desiredPosition = np.asarray(test_desiredPosition).reshape((3,1))
            logger.info( "desired position is: %s, controller gains are %d and %d. End-effector is link: %d ", test_desiredPosition, Kd, Kp, linkIndex)
            #test_vector = np.zeros(5)
            # initialize the tasks:

            test_selectionVector = data["qpController"]["positionTask"]["axis_selection"]
            test_selectionVector = np.asarray(test_selectionVector).reshape((3, 1))

            newPositionTask = positionTask.positionTask(self.robot, test_desiredPosition, test_weight, test_selectionVector, Kd, Kp, linkIndex)
            self.obj.addTask(newPositionTask)
            logger.info("initialized position task ")

        else:
            logger.info("Position task is disabled")

        test_orientationTask = data["qpController"]["orientationTask"]["enabled"]

        if(test_orientationTask):

            test_stableStill = data["qpController"]["orientationTask"]["stayStill"]
            Kp = data["qpController"]["orientationTask"]["Kp"]
            Kd = data["qpController"]["orientationTask"]["Kd"]
            linkIndex = data["qpController"]["orientationTask"]["bodyLinkNumber"]

            if(test_stableStill):
                transform = self.robot.bodynodes[linkIndex].world_transform()
                #transform = self.robot.bodynodes[linkIndex].transform()

                rotation = transform[:3, :3]
                test_desiredOrientation = pydart.utils.transformations.quaternion_from_matrix(rotation)
            else:
                test_desiredOrientation = data["qpController"]["orientationTask"]["setPoint"]
            test_weight = data["qpController"]["orientationTask"]["taskWeight"]


            #test_desiredPosition = listToArray.listToArray(test_desiredPosition)
            #test_desiredPosition = np.asarray(test_desiredPosition).reshape((3,1))
            logger.info( "desired orientation is: %s, controller gains are %d and %d. End-effector is link: %d ", test_desiredOrientation, Kd, Kp, linkIndex)
            #test_vector = np.zeros(5)
            # initialize the tasks:

            test_selectionVector = data["qpController"]["orientationTask"]["axis_selection"]
            test_selectionVector = np.asarray(test_selectionVector).reshape((3, 1))

            test_scalarWeight = data["qpController"]["orientationTask"]["quaternion_scalar_weight"]

            newOrientationTask = orientationTask.orientationTask(self.robot, test_desiredOrientation, test_weight, test_scalarWeight, test_selectionVector, Kd, Kp, linkIndex)

            self.obj.addTask(newOrientationTask)
            logger.info("initialized orientation task ")

        else:
            logger.info("Orientation task is disabled")


        test_velocityTask = data["qpController"]["velocityTask"]["enabled"]

        if(test_velocityTask):
            test_desiredVelocity = data["qpController"]["velocityTask"]["desiredVelocity"]
            Kp = data["qpController"]["velocityTask"]["Kp"]
            linkIndex = data["qpController"]["velocityTask"]["bodyLinkNumber"]
            test_desiredVelocity = np.asarray(test_desiredVelocity).reshape((3,1))
            #logger.info( "desired position is: %s, controller gains are %d and %d. End-effector is link: %d ", test_desiredVelocity, Kp, linkIndex)


            test_selectionVector = data["qpController"]["velocityTask"]["axis_selection"]
            test_selectionVector = np.asarray(test_selectionVector).reshape((3, 1))

            newVelocityTask = translationVelocityTask.translationVelocityTask(self.robot, test_desiredVelocity, test_selectionVector, Kp, linkIndex)
            self.obj.addTask(newVelocityTask)
            logger.info("initialized velocity task ")
        else:
            logger.info("Velocity task disabled")

        test_maxVelocityTask = data["qpController"]["maxVelocityTask"]["enabled"]
        if(test_maxVelocityTask):
            test_desiredDirection = data["qpController"]["maxVelocityTask"]["direction"]
            linkIndex = data["qpController"]["maxVelocityTask"]["bodyLinkNumber"]
            test_weight = data["qpController"]["maxVelocityTask"]["taskWeight"]

            newMaxVelocityTask = maxContactVelTask.maxContactVelTask(self.robot, test_desiredDirection, test_weight, linkIndex)

            self.obj.addTask(newMaxVelocityTask)
            logger.info("initialized maximizing velocity task ")

        else:
            logger.info("Maximizing velocity task disabled")

        test_maxImpactForceTask = data["qpController"]["maxImpactForceTask"]["enabled"]
        if(test_maxImpactForceTask):
            test_desiredDirection = data["qpController"]["maxImpactForceTask"]["direction"]
            linkIndex = data["qpController"]["maxImpactForceTask"]["bodyLinkNumber"]
            test_weight = data["qpController"]["maxImpactForceTask"]["taskWeight"]

            newMaxImpactForceTask = maxImpactForceTask.maxImpactForceTask(self.robot, test_desiredDirection, test_weight, linkIndex)

            self.obj.addTask(newMaxImpactForceTask)
            logger.info("initialized maximizing impact force task ")

        else:
            logger.info("Maximizing impact force task disabled")


        test_trajectoryTask = data["qpController"]["trajectoryTask"]["enabled"]

        if(test_trajectoryTask):
            test_weight = data["qpController"]["trajectoryTask"]["taskWeight"]

            Kp = data["qpController"]["trajectoryTask"]["Kp"]
            Kd = data["qpController"]["trajectoryTask"]["Kd"]

            way_points = data["qpController"]["trajectoryTask"]["way-points"]
            way_points_vector = np.zeros((3, len(way_points)))

            for ii in range(0, len(way_points)):
                way_points_vector[:,[ii]] = np.asarray(way_points[ii]).reshape((3,1))


            localEeFrame = data["qpController"]["trajectoryTask"]["end-effector-frame"]
            bodyNodeIndex = data["qpController"]["trajectoryTask"]["bodyLinkNumber"]

            if(localEeFrame):
                global_way_points_vector = self.transformToGlobalFrame(way_points_vector, bodyNodeIndex)

            else:
                global_way_points_vector = way_points_vector



            timeKnots = data["qpController"]["trajectoryTask"]["time-knots"]
            sampleNumber = data["qpController"]["trajectoryTask"]["sampleNumber"]

            newTrajectoryTask = trajectoryTask.trajectoryTask(self.robot, global_way_points_vector, timeKnots, test_weight, sampleNumber, Kp, Kd)
            self.obj.addTask(newTrajectoryTask)

            logger.info("initialized trajectory task ")
        else:
            logger.info("Trajectory task disabled")

        self.equalityConstraints = []
        logger.info( "initialized equality constraints ")

        self.inequalityConstraints = []


        self.inequalityConstraints.append(self.jointLimitConstraints)
        logger.info("initialized joint position limits inequality constraints ")


        self.inequalityConstraints.append(self.jointVelocityLimitConstraints)
        logger.info("initialized joint velocity limits inequality constraints ")

        if self.impactRobust:
            self.inequalityConstraints.append(self.robustJointLimitConstraints)
            self.inequalityConstraints.append(self.robustJointVelocityLimitConstraints)

        self.inequalityConstraints.append(self.jointAccelerationLimitConstraints)
        logger.info("initialized joint acceleration limits inequality constraints ")


        self.inequalityConstraints.append(self.torqueLimitConstraints)
        logger.info("initialized torque limits inequality constraints ")

        if self.impactRobust:
            self.inequalityConstraints.append(self.impulseTorqueLimitConstraints)
            logger.info("initialized impulse torque limits inequality constraints ")

        #self.inequalityConstraints.append(self.impactBoundConstraints)
        #logger.info("initialized impact inequality constraints ")
        
        self.equalityConstraints.append(self.impactConstraints)
        logger.info("initialized impact equality constraints ")


    def getContactStatus(self):
        return (self.contactStart and self.contactAware)


    def setContactStatus(self):
        self.contactStart = True
    def resetContactstatus(self):
        self.contactStart = False

    def transformToGlobalFrame(self, input, bodyNodeIndex):

        transform = self.robot.bodynodes[bodyNodeIndex].world_transform()

        #transform_inv = np.linalg.inv(transform)
        #rotation = transform_inv[:3, :3]
        #translation = transform_inv[:3, 3]

        rotation = transform[:3, :3]
        translation = transform[:3, 3]

        translation = np.reshape(translation, (3,1))
        output = np.zeros(input.shape)

        for ii in range(0, input.shape[1]):

            output[:,[ii]] = rotation.dot(input[:,[ii]]) + translation

        return output

    def update(self, impactEstimator):
        self.contact.update()

        for ii in range(0, len(self.inequalityConstraints)):
            self.inequalityConstraints[ii].update(impactEstimator)

        for ii in range(0, len(self.equalityConstraints)):
            self.equalityConstraints[ii].update(impactEstimator)

        for ii in range(0,  len(self.obj.tasks)):
            self.obj.tasks[ii].update()


    def solve(self, impactEstimator):
        self.update(impactEstimator)

        [Q, P, C] = self.obj.calcMatricies( self.getContactStatus(), self.contact)

        [G, H] = self.inequalityConstraints[0].calcMatricies(self.getContactStatus(), self.contact)
        for ii in range(1, len(self.inequalityConstraints)):
            [G_ii, H_ii] = self.inequalityConstraints[ii].calcMatricies(self.getContactStatus(), self.contact)
            G = np.concatenate((G, G_ii), axis=0)
            H = np.concatenate((H, H_ii), axis=0)


        if(self.getContactStatus()):
            [G_contact, H_contact] = self.contact.calcContactConstraintMatrices()

            G = np.concatenate((G, G_contact), axis=0)
            H = np.concatenate((H, H_contact), axis=0)

        if len(self.equalityConstraints) > 0:
            [A, B] = self.equalityConstraints[0].calcMatricies(self.getContactStatus(), self.contact)
                          
            for ii in range(1, len(self.equalityConstraints)):
                [A_ii, B_ii] = self.equalityConstraints[ii].calcMatricies(self.getContactStatus(), self.contact)
                A = np.concatenate((A, A_ii), axis=0)
                B = np.concatenate((B, B_ii), axis=0)

            sol = self.cvxopt_solve_qp(Q, P.T, G, H, A, B)
        else:
            sol = self.cvxopt_solve_qp(Q, P.T, G, H)

        if sol is -1:
            return [-1, -1, -1]



            #sol = solve_qp(Q, P.T, G, H, solver='mosek')
        # We only take the first 6 joint accelerations
        if  self.getContactStatus():
        #if True:

            solution = sol.reshape(2*self.robot.ndofs + self.contact.Nc, 1)
            sol_ddq = solution[:self.robot.ndofs]
            sol_delta_dq = solution[self.robot.ndofs:2*self.robot.ndofs] # How to extract middle ones?
            sol_weights = solution[2*self.robot.ndofs:] # Take the last ones
        else:

            solution = sol.reshape(2 * self.robot.ndofs, 1)
            sol_ddq = solution[:self.robot.ndofs]
            sol_delta_dq = solution[self.robot.ndofs:]
            sol_weights = np.zeros((self.contact.Nc, 1))

        self.contact.updateWeights(sol_weights)

        return [sol_ddq, sol_delta_dq, sol_weights]


    def cvxopt_matrix(self, M):
        if type(M) is np.ndarray:
            return matrix(M)
        elif type(M) is spmatrix or type(M) is matrix:
            return M
        coo = M.tocoo()
        return spmatrix(
            coo.data.tolist(), coo.row.tolist(), coo.col.tolist(), size=M.shape)

    def cvxopt_solve_qp(self, P, q, G=None, h=None, A=None, b=None, initvals=None):

        args = [self.cvxopt_matrix(P), self.cvxopt_matrix(q)]
        if G is not None:
            args.extend([self.cvxopt_matrix(G), self.cvxopt_matrix(h)])
            if A is not None:
                args.extend([self.cvxopt_matrix(A), self.cvxopt_matrix(b)])

        try:
            sol = qp(*args, initvals=initvals)
        except ValueError:
            logging.error("QP is infeasible ")
            self.robot.controller.logData()
            #temp = np.zeros((q.shape[0],1))
            return -1

        if 'optimal' not in sol['status']:
            logging.error("QP fails, the status are: %s", sol )
            logging.error("The s variable is, %s", sol['s'])
            self.robot.controller.logData()
            temp = np.zeros((q.shape[0], 1))
            return -1

        return np.array(sol['x']).reshape((q.shape[0],))




def jointAccToTau(robot, jointAcc):
    """!@brief
    Calculates the torque corresponding to the solved acceleration.
    @param jointAcc The resolved acceleration from the QP.
    """

    # Use the end-effector Jacobian
    J = robot.bodynodes[-1].world_jacobian()

    # It seems that we can work with this equality if there is no forces
    #print "coriolis and gravities are: ", '\n', robot.coriolis_and_gravity_forces()

    # print "test J is: ",'\n', J.transpose().dot(robot.constraint_forces().reshape((robot.ndofs,1)))

    tau = robot.M.dot(jointAcc) + robot.coriolis_and_gravity_forces().reshape((robot.ndofs, 1)) - robot.constraint_forces().reshape((robot.ndofs,1))
    # J.transpose().dot(robot.constraint_forces().reshape((robot.ndofs,1)))


    return tau



if __name__ == "__main__":
    print('Hello, PyDART!')

    pydart.init()

    test_world = pydart.World(1.0 / 2000.0, "../data/skel/two_cubes.skel")

    test_robot = test_world.add_skeleton("../data/KR5/KR5_sixx_R650.urdf")

    with open('../config/impact_one.json') as file:
        qpData = json.load(file)

    test_qp = manipulatorQP(test_robot, qpData)

    solution = test_qp.solve()
    print "The solution is: ", '\n', solution

    test_tau = jointAccToTau(test_robot, solution)

    print "The torque should be: ", '\n', test_tau
