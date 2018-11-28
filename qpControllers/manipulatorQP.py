import pydart2 as pydart

import _pydart2_api as papi

import numpy as np

from cvxopt import matrix, solvers

from manipulatorConstraints import jointPositionLimits, jointVelocityConstraints, jointAccelerationConstraints, torqueLimitConstraints

from qpControllers import qpObj

from manipulatorTasks import positionTask
from manipulatorTasks import orientationTask
from manipulatorTasks import translationVelocityTask
from manipulatorTasks import trajectoryTask
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
        self.jointLimitConstraints = jointPositionLimits.jointLimitConstraints(self.robot, dt)

        self.jointVelocityLimitConstraints = jointVelocityConstraints.jointVelocityLimitConstraints(self.robot, dt)
        self.jointAccelerationLimitConstraints = jointAccelerationConstraints.jointAccelerationLimitConstraints(self.robot)

        # The joint velocity limtis is not given by skeleton
        # self.jointsVUpper =
        # self.jointsVLower =

        upper = data["qpController"]["torqueLimits"]["upper"]
        lower = data["qpController"]["torqueLimits"]["lower"]

        self.torqueLower = np.reshape(np.asarray(lower), (self.dof, 1))
        self.torqueUpper = np.reshape(np.asarray(upper), (self.dof, 1))
        logger.info("torque upper limit is: %s ",  self.torqueUpper)
        logger.info("torque lower limit is: %s ", self.torqueLower)

        # print "torque upper limit is: ", self.torqueUpper
        # print "torque lower limit is: ", self.torqueLower

        # self.torqueLower = np.reshape(self.robot.tau_lower, (self.dof, 1))
        # self.torqueUpper = np.reshape(self.robot.tau_upper, (self.dof, 1))

        self.torqueLimitConstraints = torqueLimitConstraints.torqueLimitConstraints(self.robot, self.torqueUpper, self.torqueLower)

        logger.info("initialized constraints ")


        # initialize objective function:
        test_jointUnitWeight = data["qpController"]["jointUnitWeight"]
        self.obj = qpObj.qpObj(self.robot, test_jointUnitWeight)
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

        self.equalityConstaints = []
        logger.info( "initialized equality constraints ")

        self.inequalityConstraints = []


        self.inequalityConstraints.append(self.jointLimitConstraints)
        logger.info("initialized joint position limits inequality constraints ")

        self.inequalityConstraints.append(self.jointVelocityLimitConstraints)
        logger.info("initialized joint velocity limits inequality constraints ")

        #self.inequalityConstraints.append(self.jointAccelerationLimitConstraints)
        #logger.info("initialized joint acceleration limits inequality constraints ")


        self.inequalityConstraints.append(self.torqueLimitConstraints)
        logger.info("initialized torque limits inequality constraints ")

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

        for ii in range(0, len(self.inequalityConstraints)):
            self.inequalityConstraints[ii].update(impactEstimator)

        for ii in range(0, len(self.equalityConstaints)):
            self.equalityConstaints[ii].update()

        for ii in range(0,  len(self.obj.tasks)):
            self.obj.tasks[ii].update()

    def solve(self, impactEstimator):
        self.update(impactEstimator)

        [Q, P, C] = self.obj.calcMatricies()

        [G, H] = self.inequalityConstraints[0].calcMatricies()
        for ii in range(1, len(self.inequalityConstraints)):
            [G_ii, H_ii] = self.inequalityConstraints[ii].calcMatricies()
            G = np.concatenate((G, G_ii), axis=0)
            H = np.concatenate((H, H_ii), axis=0)

        #print "G is: ", '\n', G
        #print "H is: ", '\n', H

        if len(self.equalityConstaints) > 0:
            [A, B] = self.equalityConstraints[0].calcMatricies()

            for ii in range(1, len(self.equalityConstaints)):
                [A_ii, B_ii] = self.equalityConstaints[ii].calcMatricies()
                A = np.concatenate((A, A_ii), axis=0)
                B = np.concatenate((B, B_ii), axis=0)
            sol = self.cvxopt_solve_qp(Q, P, G, H, A, B)

        else:
            sol = self.cvxopt_solve_qp(Q, P.T, G, H)


            #sol = solve_qp(Q, P.T, G, H, solver='mosek')
        # We only take the first 6 joint accelerations
        solution = sol.reshape(2*self.robot.ndofs, 1)
        return solution[:6]


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
            return None

        if 'optimal' not in sol['status']:
            logging.error("QP fails, the status are: %s", sol )
            logging.error("The s variable is, %s", sol['s'])
            self.robot.controller.logData()
            return None
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
