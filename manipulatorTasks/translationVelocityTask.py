import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array
from qpControllers import qpObj
import logging

class translationVelocityTask:
    def __init__(self, skel, desiredTranslationVelocity, selectionVector=None, Kp=None, bodyNodeIndex=None):

        logger = logging.getLogger(__name__)
        self.robot = skel

        if Kp is None:
            self.Kp = 10
        else:
            self.Kp = Kp

        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex

        if desiredTranslationVelocity is None:
            raise Exception("Desired translationVelocity is not set")
        else:
            self.desiredTranslationVelocity = desiredTranslationVelocity

        if selectionVector is None:
            logger.warning("Selection matrix is identity")
            self.selectionMatrix = np.identity(3)
        else:
            self.selectionMatrix = np.identity(3)
            self.selectionMatrix[0, 0] = selectionVector[0]
            self.selectionMatrix[1, 1] = selectionVector[1]
            self.selectionMatrix[2, 2] = selectionVector[2]

        self.error = np.zeros((3,1))
        self.update()

    def update(self):
        transform = self.robot.bodynodes[-1].world_transform()
        robot_ee_translation = transform[:3, 3]

        self.desiredPosition = robot_ee_translation.reshape((3,1)) + self.desiredTranslationVelocity*self.robot.world.dt

    def calcMatricies(self):

        newJacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian = self.selectionMatrix.dot(newJacobian)

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.selectionMatrix.dot(newJacobian_dot)

        # transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        # translation = transform[[0,1,2],3].reshape((3,1))

        #print "The transform is: ", transform, " shape: ", transform.shape
        #print "The translation is: ", translation

        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        self.error = self.selectionMatrix.dot(newJacobian.dot(dq) - self.desiredTranslationVelocity)

        logger = logging.getLogger(__name__)
        logger.debug('The position task error is: %s ', self.error)

        #print "The position task error is: ", '\n', error
        constant = (newJacobian_dot + self.Kp * newJacobian).dot(dq) - self.Kp * self.selectionMatrix.dot(self.desiredTranslationVelocity)
        #constant = (newJacobian_dot + self.Kp*newJacobian).dot(dq) - self.Kp*error
        #2*self.Kw*newJacobian.dot(self.robot.dq) + (translation - self.desiredPosition)


        Q = newJacobian.T.dot(newJacobian)

        P = 2*constant.T.dot(newJacobian)

        C = constant.T.dot(constant)

        #return [newJacobian, newJacobian_dot, Q, P, C]
        return [Q, P, C]




if __name__ == "__main__":

    print('Hello, PyDART!')

    pydart.init()

    test_world = pydart.World(1.0 / 2000.0, "../data/skel/two_cubes.skel")

    test_robot = test_world.add_skeleton("../data/KR5/KR5_sixx_R650.urdf")



    test_desiredVelocity = array([0.1, 0.0, 0.0]).reshape((3,1))

    test_task = translationVelocityTask(test_robot, test_desiredVelocity)

    [Q, P, C] = test_task.calcMatricies()

    # print "The jacobian is: ", '\n', jacobian
    # print "The jacobian derivative is: ", '\n', jacobian_dot
    print "The Q matrix is: ", '\n', Q
    print "The P matrix is: ", '\n', P
    print "The C matrix is: ", '\n', C

    test_obj = qpObj.qpObj(test_robot)
    test_obj.addTask(test_task)


    print "The weight matrix is: ", '\n', test_obj.dofWeightMatrix
    print "The numer of tasks is: ", test_obj.numTasks()


    [Q_obj, P_obj, C_obj] = test_obj.calcMatricies()
    print "The Q_obj matrix is: ", '\n', Q_obj
    print "The P_obj matrix is: ", '\n', P_obj
    print "The C_obj matrix is: ", '\n', C_obj
