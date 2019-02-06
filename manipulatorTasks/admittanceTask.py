import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array
from worlds import cubes_KR_R650
from controllers import gravityCompensationController
from qpControllers import qpObj
from contact import qpContact

import logging


class admittanceTask:
    """!@brief
    @class admittanceTask
    It generates a desired force with joint acceleration.
    """
    def __init__(self,skel, desiredForce, weight, selectionVector=None, velocityControl=False, desiredVelocity=None, Kd=None, Kf=None, Ki = None, bodyNodeIndex=None, qpForceRegulating=True):
        logger = logging.getLogger(__name__)

        self.robot = skel

        if Kd is None:
            self.Kd = 10
        else:
            self.Kd = Kd

        if Kf is None:
            self.Kf = 10
        else:
            self.Kf = Kf

        if Ki is None:
            self.Ki = 100
        else:
            self.Ki = Ki

        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex

        if desiredForce is None:
            raise Exception("Desired force is not set")
        else:
            self.desiredForce = desiredForce

        if weight is None:
            raise Exception("task weight is not set")
        else:
            self.weight = weight
            
        if selectionVector is None:
            logger.warning("Selection matrix is identity")
            self.selectionMatrix = np.identity(3)
        else:
            self.selectionMatrix = np.identity(3)
            self.selectionMatrix[0, 0] = selectionVector[0]
            self.selectionMatrix[1, 1] = selectionVector[1]
            self.selectionMatrix[2, 2] = selectionVector[2]

            

        self.velocityControl = velocityControl

        if desiredVelocity is None:
            self.desiredVelocity = array([0.0, 0.0, 0.0]).reshape((3,1))
        else:
            self.desiredVelocity = desiredVelocity


        self.contactCheck = False
        self.error = np.zeros((3, 1))
        self.forceErrorIntegral = np.zeros((3, 1))
        self.qpForceErrorIntegral = np.zeros((3, 1))
        self.equivalentForceVector = np.zeros((3, 1))

        # If we should use QP force to regulate the contact force task or not
        self.qpForceRegulating = qpForceRegulating


    def update(self):
        # check if we are in contact
        #self.robot.world.contact
        pass

    def calcQPAdmittaceMatricies(self, qpContact):

        qp_Ki = 10
        qp_Kf = 0.5

        newJacobian_linear = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian_linear = self.selectionMatrix.dot(newJacobian_linear)

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.selectionMatrix.dot(newJacobian_dot)

        # Calculating the equivalent end-effector force
        equivalentForce = np.linalg.pinv(newJacobian_linear.T).dot(self.robot.constraint_forces())

        f_qp = qpContact.getContactForce()
        qpForceError = self.selectionMatrix.dot(f_qp - self.desiredForce)
        
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        constant = newJacobian_dot.dot(dq) + qp_Kf * qpForceError + qp_Ki*self.qpForceErrorIntegral

        self.qpForceErrorIntegral = self.qpForceErrorIntegral + qp_Ki*qpForceError*self.robot.world.dt

        Q = newJacobian_linear.T.dot(newJacobian_linear)
        Q = np.block([
            [Q,          np.zeros((self.robot.ndofs, self.robot.ndofs))],
            [np.zeros((self.robot.ndofs, self.robot.ndofs)), np.zeros((self.robot.ndofs, self.robot.ndofs))]
        ])

        P = 2 * constant.T.dot(newJacobian_linear)
        zero_vector = np.zeros((1, self.robot.ndofs))
        P = np.concatenate((P, zero_vector), axis=1)

        C = constant.T.dot(constant)

        return [Q, P, C]

    def calcQPRegulatingMatricies(self, qpContact):
        # Minimize the error: ||f_qp - f_desired||_2
        constant = 1.0
        f_qp = qpContact.getContactForce()
        #force_Q = constant*f_qp.transpose().dot(f_qp)

        #force_Q = constant*qpContact.getContactGenerationMatrix().transpose().dot(qpContact.getContactGenerationMatrix())
        #force_Q = (force_Q.transpose() + force_Q)/2
        force_Q = np.identity(4)
        #force_Q = np.zeros((4,4))

        force_P = - constant*(self.desiredForce.transpose()).dot(qpContact.getContactGenerationMatrix())

        C = constant*self.desiredForce.transpose().dot(self.desiredForce)

        return [force_Q, force_P, C]


    def basicCase(self, useContactVariables, qpContact):
        #newJacobian = self.robot.bodynodes[self.bodyNodeIndex].world_jacobian()
        newJacobian_linear = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()
        newJacobian_linear = self.selectionMatrix.dot(newJacobian_linear)

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()
        newJacobian_dot = self.selectionMatrix.dot(newJacobian_dot)

        # Calculating the equivalent end-effector force
        equivalentForce = np.linalg.pinv(newJacobian_linear.T).dot(self.robot.constraint_forces())
        


        # I am not sure if this is the right way to do it.
        self.equivalentForceVector = - equivalentForce[0:3:1].reshape((3,1))
        self.currentForce = self.equivalentForceVector

        if self.qpForceRegulating:
            f_qp = qpContact.getContactForce()
            # f_target = -(f_qp - self.desiredForce)
            # forceError = self.selectionMatrix.dot(self.equivalentForceVector - f_target)
            forceError = self.selectionMatrix.dot(self.equivalentForceVector - f_qp)
        else:
            forceError = self.selectionMatrix.dot(self.equivalentForceVector - self.desiredForce)

        print "The desired force is: ", self.desiredForce.T
        # print "The current QP force is: ", qpContact.getContactForce().T
        # print "The current target force is: ", f_qp.T
#        print "The current target force is: ", f_target.T
        print "The current sensor force is: ", self.equivalentForceVector.T
        print "The force error is: ", forceError.T



        self.error = forceError

        #
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        constant = newJacobian_dot.dot(dq) + self.Kf * forceError + self.Ki*self.forceErrorIntegral

        self.forceErrorIntegral = self.forceErrorIntegral + self.Ki*forceError*self.robot.world.dt

        Q = newJacobian_linear.T.dot(newJacobian_linear)
        Q = np.block([
            [Q,          np.zeros((self.robot.ndofs, self.robot.ndofs))],
            [np.zeros((self.robot.ndofs, self.robot.ndofs)), np.zeros((self.robot.ndofs, self.robot.ndofs))]
        ])

        P = 2 * constant.T.dot(newJacobian_linear)
        zero_vector = np.zeros((1, self.robot.ndofs))
        P = np.concatenate((P, zero_vector), axis=1)

        C = constant.T.dot(constant)

        if(useContactVariables):

            
            QP_original_weight = 1.0

            QP_size = 2*self.robot.ndofs
            contact_size = qpContact.Nc
            Q_new  = np.zeros((QP_size + contact_size, QP_size + contact_size))
            Q_new[:QP_size, :QP_size] = Q*QP_original_weight # write the old info
            
            P_new = np.zeros((1, QP_size + contact_size))
            P_new[0, :QP_size] = P*QP_original_weight

            if self.qpForceRegulating:
                weights = 10
                [force_Q, force_P, force_C] = self.calcQPRegulatingMatricies(qpContact)
                Q_new[2 * self.robot.ndofs:, 2 * self.robot.ndofs:] = force_Q*weights
                P_new[:, 2 * self.robot.ndofs:] = force_P*weights

            # if self.qpForceRegulating:
            #     [force_Q, force_P, force_C] = self.calcQPAdmittaceMatricies(qpContact)
            #     weight_qpForce = 1.0
            #     # Q_new[2 * self.robot.ndofs:, 2 * self.robot.ndofs:] = force_Q
            #     # P_new[:, 2 * self.robot.ndofs:] = force_P
            #     Q_new[:2 * self.robot.ndofs, :2 * self.robot.ndofs] += force_Q*weight_qpForce
            #     P_new[:, :2 * self.robot.ndofs] += force_P*weight_qpForce


            Q = Q_new
            P = P_new

        return [self.weight*Q, self.weight*P, self.weight*C]


    def motionCase(self, useContactVariables, qpContact):
        pass

    def calcMatricies(self, useContactVariables, qpContact):
        if self.velocityControl:
            return self.motionCase(useContactVariables, qpContact)
        else:
            return self.basicCase(useContactVariables, qpContact)

if __name__ == "__main__":

    print('Hello, PyDART!')

    pydart.init()
    world_file = "../data/skel/two_cubes.skel"
    robot_file = "../data/KR5/KR5_sixx_R650.urdf"

    test_world = cubes_KR_R650.cubeKR5World_admittance_task( world_file, robot_file)

    test_robot = test_world.skeletons[-1]



    desiredForce = np.reshape([10.0, 0.0, 0.0], (3,1))
    # test_desiredPosition = array([0.1, 0.2, 0.3]).reshape((3,1))
    taskWeight = 1000

    test_task = admittanceTask(test_robot, desiredForce, taskWeight, Kf=100, Ki=20, bodyNodeIndex=-1)

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


