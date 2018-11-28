import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array
from worlds import cubes_KR_R650
from controllers import gravityCompensationController
from qpControllers import qpObj

import logging

class contactAdmittanceTask:
    """!@brief
    It keeps a point contact and generates a desired force by generating joint acceleration.
    """
    def __init__(self,skel, desiredForce, weight, desiredPosition, Kd, Kp, Kf, Ki, selectionVector=None, bodyNodeIndex=None):

        logger = logging.getLogger(__name__)

        self.robot = skel

        if desiredForce is None:
            raise Exception("Desired force is not set")
        else:
            self.desiredForce = desiredForce

        if desiredPosition is None:
            raise Exception("Desired position is not set")
        else:
            self.desiredPosition = desiredPosition


        if weight is None:
            raise Exception("task weight is not set")
        else:
            self.weight = weight

        self.Kd = Kd
        self.Kp = Kp
        self.Kf = Kf
        self.Ki = Ki


        if selectionVector is None:
            logger.warning("Selection matrix is identity")
            self.selectionMatrix = np.identity(3)
        else:
            self.selectionMatrix = np.identity(3)
            self.selectionMatrix[0, 0] = selectionVector[0]
            self.selectionMatrix[1, 1] = selectionVector[1]
            self.selectionMatrix[2, 2] = selectionVector[2]

        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex


        self.contactCheck = False
        self.error = np.zeros((3, 1))

        self.forceErrorIntegral = np.zeros((3, 1))
        self.equivalentForceVector = np.zeros((3, 1))
        # We set it to be zero always
        self.desiredTranslationVelocity = np.zeros((3, 1))

    def update(self):
        # check if we are in contact
        #self.robot.world.contact
        pass


    def calcMatricies(self):

        # Let's use J_ac = body Jacobian
        # transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        # rotation = transform[:3, :3]
        # newJacobian_linear = rotation.dot(self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian())

        # Use the linear Jacobian:

        newJacobian_linear = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian_linear = self.selectionMatrix.dot(newJacobian_linear)

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.selectionMatrix.dot(newJacobian_dot)

        ################## Construct the force error
        # Calculating the equivalent end-effector force
        equivalentForce = np.linalg.pinv(newJacobian_linear.T).dot(self.robot.constraint_forces())
        # I am not sure if this is the right way to do it.
        self.equivalentForceVector = - equivalentForce[0:3:1].reshape((3,1))

        forceError = self.selectionMatrix.dot(self.equivalentForceVector - self.desiredForce)
        print "The desired force is: ", self.desiredForce.T
        print "The current force is: ", self.equivalentForceVector.T
        print "The force error is: ", forceError.T

        self.error = forceError
        ################# Construct the position error
        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        translation = transform[[0,1,2],3].reshape((3,1))


        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))
        positionError = self.selectionMatrix.dot(translation - self.desiredPosition )
        velocityError = self.selectionMatrix.dot(newJacobian_linear.dot(dq) - self.desiredTranslationVelocity)

        constant = newJacobian_dot.dot(dq) + self.Kd*velocityError + self.Kp*positionError + self.Kf * forceError + self.Ki*self.forceErrorIntegral

        self.forceErrorIntegral = self.forceErrorIntegral + self.Ki*forceError*self.robot.world.dt

        Q = newJacobian_linear.T.dot(newJacobian_linear)

        P = 2 * constant.T.dot(newJacobian_linear)

        C = constant.T.dot(constant)

        return [self.weight*Q, self.weight*P, self.weight*C]
if __name__ == "__main__":

    print('Hello, PyDART!')

    pydart.init()
    world_file = "../data/skel/two_cubes.skel"
    robot_file = "../data/KR5/KR5_sixx_R650.urdf"

    test_world = cubes_KR_R650.cubeKR5World_admittance_task( world_file, robot_file)

    test_robot = test_world.skeletons[-1]



    desiredForce = np.reshape([10.0, 0.0, 0.0], (3,1))
    test_desiredPosition = array([1.0, 0.0, 0.0]).reshape((3,1))
    taskWeight = 1000

    test_task = contactAdmittanceTask(test_robot, desiredForce, taskWeight, test_desiredPosition, Kd= 10, Kp= 10, Kf=100, Ki=20, bodyNodeIndex=-1)

    [Q, P, C] = test_task.calcMatricies()

    # print "The jacobian is: ", '\n', jacobian
    # print "The jacobian derivative is: ", '\n', jacobian_dot
    print "The Q matrix is: ", '\n', Q
    print "The P matrix is: ", '\n', P
    print "The C matrix is: ", '\n', C

    test_obj = qpObj.qpObj(test_robot, 100)
    test_obj.addTask(test_task)

    print "The weight matrix is: ", '\n', test_obj.dofWeightMatrix
    print "The numer of tasks is: ", test_obj.numTasks()

    [Q_obj, P_obj, C_obj] = test_obj.calcMatricies()
    print "The Q_obj matrix is: ", '\n', Q_obj
    print "The P_obj matrix is: ", '\n', P_obj
    print "The C_obj matrix is: ", '\n', C_obj


