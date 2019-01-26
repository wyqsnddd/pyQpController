import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array

from worlds import cubes_KR_R650
from controllers import gravityCompensationController
from qpControllers import qpObj

import logging


class orientationTask:
    """!@brief
    We use unit quaternion and second order inverse dynamics to fulfill an orientation task
    """

    def __init__(self, skel, desiredOrientation, taskWeight, scalarWeight, selectionVector=None, Kd=None, Kp=None, bodyNodeIndex=None):

        logger = logging.getLogger(__name__)
        self.robot = skel

        if Kd is None:
            self.Kd = 10
        else:
            self.Kd = Kd

        if Kp is None:
            self.Kp = 10
        else:
            self.Kp = Kp

        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex

        if desiredOrientation is None:
            raise Exception("Desired orientation is not set")
        else:
            self.desiredOrientation = desiredOrientation

        if taskWeight is None:
            raise Exception("Task weight is not set")
        else:
            self.taskWeight = taskWeight

        if selectionVector is None:
            logger.warning("Selection matrix is identity")
            self.selectionMatrix = np.identity(4)
        else:
            self.selectionMatrix = np.identity(4)
            self.selectionMatrix[1, 1] = selectionVector[0]
            self.selectionMatrix[2, 2] = selectionVector[1]
            self.selectionMatrix[3, 3] = selectionVector[2]

        self.error = np.zeros((3, 1))

        self.quat_desired_m = self.Q_bar(self.desiredOrientation)

        self.q_scalar_weight = scalarWeight
        # construct the quaternion multiplication matrix:

        # Error checking:
        self.current_quat_d_last = np.zeros((4,1))
        self.current_quat_dd_last = np.zeros((4, 1))

        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        rotation = transform[:3,:3].reshape((3,3))
        self.iniRotation = rotation
        self.rotation_last = self.iniRotation

        self.error_last = np.zeros((4,1))
        self.error_v_last = np.zeros((4,1))

        
        self.current_quat_last = pydart.utils.transformations.quaternion_from_matrix(rotation)
        #self.current_quat_last = pydart.utils.transformations.quaternion_conjugate(self.current_quat_last)

        # end

    def Q(self, quaternion):

        Q = np.identity(4)

        Q[0, 0] = quaternion[0]
        Q[1, 0] = quaternion[1]
        Q[2, 0] = quaternion[2]
        Q[3, 0] = quaternion[3]

        Q[0, 1] = -quaternion[1]
        Q[1, 1] = quaternion[0]
        Q[2, 1] = -quaternion[3]
        Q[3, 1] = quaternion[2]

        Q[0, 2] = -quaternion[2]
        Q[1, 2] = quaternion[3]
        Q[2, 2] = quaternion[0]
        Q[3, 2] = -quaternion[1]

        Q[0, 3] = -quaternion[3]
        Q[1, 3] = -quaternion[2]
        Q[2, 3] = quaternion[1]
        Q[3, 3] = quaternion[0]

        return Q

    def Q_bar(self,quaternion):

        Q_bar = np.identity(4)

        Q_bar[0, 0] = quaternion[0]
        Q_bar[1, 0] = -quaternion[1]
        Q_bar[2, 0] = -quaternion[2]
        Q_bar[3, 0] = -quaternion[3]

        Q_bar[0, 1] = quaternion[1]
        Q_bar[1, 1] = quaternion[0]
        Q_bar[2, 1] = quaternion[3]
        Q_bar[3, 1] = -quaternion[2]

        Q_bar[0, 2] = quaternion[2]
        Q_bar[1, 2] = -quaternion[3]
        Q_bar[2, 2] = quaternion[0]
        Q_bar[3, 2] = quaternion[1]

        Q_bar[0, 3] = quaternion[3]
        Q_bar[1, 3] = quaternion[2]
        Q_bar[2, 3] = -quaternion[1]
        Q_bar[3, 3] = quaternion[0]
        return Q_bar


    def W(self, quaternion):

        quat_W = np.zeros([3,4])

        quat_W[0, 0] = -quaternion[1]
        quat_W[1, 0] = -quaternion[2]
        quat_W[2, 0] = -quaternion[3]

        quat_W[0, 1] = quaternion[0]
        quat_W[1, 1] = quaternion[3]
        quat_W[2, 1] = -quaternion[2]

        quat_W[0, 2] = -quaternion[3]
        quat_W[1, 2] = quaternion[0]
        quat_W[2, 2] = quaternion[1]

        quat_W[0, 3] = quaternion[2]
        quat_W[1, 3] = -quaternion[1]
        quat_W[2, 3] = quaternion[0]

        return quat_W


    def W_prime(self, quaternion):

        quat_W_prime = np.zeros([3, 4])

        quat_W_prime[0, 0] = quaternion[1]
        quat_W_prime[1, 0] = quaternion[2]
        quat_W_prime[2, 0] = quaternion[3]

        quat_W_prime[0, 1] = quaternion[0]
        quat_W_prime[1, 1] = -quaternion[3]
        quat_W_prime[2, 1] = quaternion[2]

        quat_W_prime[0, 2] = quaternion[3]
        quat_W_prime[1, 2] = quaternion[0]
        quat_W_prime[2, 2] = -quaternion[1]

        quat_W_prime[0, 3] = -quaternion[2]
        quat_W_prime[1, 3] = quaternion[1]
        quat_W_prime[2, 3] = quaternion[0]

        return quat_W_prime

    def update(self):
        pass


    def testQuaternionRates(self):
        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        rotation = transform[:3,:3].reshape((3,3))

        current_quat = pydart.utils.transformations.quaternion_from_matrix(rotation)


        current_quat_d = (current_quat - self.current_quat_last)/self.robot.world.dt

        current_quat_dd = (current_quat_d - self.current_quat_d_last) / self.robot.world.dt

        print "The current end-effector quaternion rate is: ", '\n', current_quat_d.transpose()

        test_dimension_Matrix = np.zeros((4,3))
        test_dimension_Matrix[1:4, :] = np.identity(3)

        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        angular_jac = self.robot.bodynodes[self.bodyNodeIndex].angular_jacobian()



        test_Q = 0.5*self.Q(current_quat)

        temp_1 = test_Q.dot(test_dimension_Matrix)

        current_quat_d_calc = temp_1.dot(angular_jac.dot(dq))
        print "The calculated current end-effector quaternion rate is: ", '\n', current_quat_d_calc.transpose()

        current_quat_d_calc = 0.5*(self.W(current_quat).transpose()).dot(angular_jac.dot(dq))

        #print "The calculated current end-effector quaternion rate is: ", '\n', current_quat_d_calc.transpose()

        print "The current end-effector quaternion acc is: ", '\n', current_quat_dd.transpose()

        ddq = (self.robot.ddq).reshape((self.robot.ndofs, 1))

        angular_jac_dot = self.robot.bodynodes[self.bodyNodeIndex].angular_jacobian_deriv()
        current_quat_dd_calc = temp_1.dot(angular_jac_dot.dot(dq) + angular_jac.dot(ddq))
        print "The calculated current end-effector quaternion acc is: ", '\n', current_quat_dd_calc.transpose()
        current_quat_dd_calc = 0.5*(self.W(current_quat).transpose()).dot(angular_jac_dot.dot(dq) + angular_jac.dot(ddq))
        print "The calculated current end-effector quaternion acc is: ", '\n', current_quat_dd_calc.transpose()

        current_quat_con = pydart.utils.transformations.quaternion_conjugate(current_quat)
        print "Test conjugate: ", (self.Q(current_quat_con).dot(current_quat)).transpose()
        print "Test conjugate: ", (self.Q_bar(current_quat).dot(current_quat)).transpose()

        self.current_quat_last = current_quat
        self.current_quat_d_last = current_quat_d


    def vee(self, input):
        w = np.zeros((3,1))
        w[0] = 0.5*(input[2,1] - input[1, 2])
        w[1] = 0.5*(-input[2,0] + input[0, 2])
        w[2] = 0.5*(input[1,0] - input[0, 1])
        
        return w
    
    def testErrorRates(self):

        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        rotation = transform[:3,:3].reshape((3,3))

        rotation_rate = (rotation - self.rotation_last)/self.robot.world.dt

        body_velocity_skew = (rotation.transpose()).dot(rotation_rate)
        print "the body velocity is: ", '\n', self.vee(body_velocity_skew).transpose()

        spatial_velocity_skew = rotation_rate.dot(rotation.transpose())
        print "the spatial velocity is: ", '\n', self.vee(spatial_velocity_skew).transpose()

        angular_jacobian = self.robot.bodynodes[self.bodyNodeIndex].angular_jacobian()
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        calc_w = angular_jacobian.dot(dq)
        print "the calculated angular velocity is: ", '\n', calc_w.transpose()

        current_quat = pydart.utils.transformations.quaternion_from_matrix(rotation)

        current_quat_d = (current_quat - self.current_quat_last) / self.robot.world.dt
        self.current_quat_last = current_quat
        print "the predicted spatial velocity is: ", '\n', 2*self.Q(current_quat).transpose().dot(current_quat_d)


        current_quat = pydart.utils.transformations.quaternion_from_matrix(rotation)
        current_quat_con = pydart.utils.transformations.quaternion_conjugate(current_quat)
        current_quat_con = np.reshape(current_quat_con, (4,1))

        error = self.quat_desired_m.dot(current_quat_con)
        
        error_v = (error - self.error_last)/self.robot.world.dt



        print "the current error v is: ", '\n', error_v.transpose()

        temp_id = -np.identity(4)
        temp_id[0, 0] = 1

        block_one = (self.quat_desired_m.dot(temp_id)).dot(self.W(current_quat).transpose())

        error_v_calc = 0.5*block_one.dot(calc_w)
        
        print "the calculated error v is: ", '\n', error_v_calc.transpose()
        
        error_acc = (error_v - self.error_v_last)/self.robot.world.dt        
        print "the current error acc is: ", '\n', error_acc.transpose()

        angular_jacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].angular_jacobian_deriv()
        
        ddq = (self.robot.ddq).reshape((self.robot.ndofs, 1))
        error_acc_calc = 0.5*block_one.dot(angular_jacobian_dot.dot(dq) + angular_jacobian.dot(ddq) )
        print "the calculated error acc is: ", '\n', error_acc_calc.transpose()

        error_test_1 = self.quat_desired_m.dot(current_quat_con)
        error_test_2 = (self.quat_desired_m.transpose()).dot(current_quat)

        print "old error is: ", error_test_1.transpose()
        print "new error is: ", error_test_2.transpose()

        # update
        self.rotation_last = rotation
        self.error_last = error
        self.error_v_last = error_v

    def calcMatricies(self, useContactVariables, qpContact):
        #self.testQuaternionRates()
        #self.testErrorRates()

        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        #transform = self.robot.bodynodes[self.bodyNodeIndex].transform()

        rotation = transform[:3,:3].reshape((3,3))

        current_quat = pydart.utils.transformations.quaternion_from_matrix(rotation)
        current_quat = np.reshape(current_quat, (4,1))

        #current_quat_con = pydart.utils.transformations.quaternion_conjugate(current_quat)
        #current_quat_con = np.reshape(current_quat_con, (4,1))

        temp_id = -np.identity(4)
        temp_id[0, 0] = 1

        block_one = (self.quat_desired_m.dot(temp_id)).dot(self.W(current_quat).transpose())
        #block_one = (self.quat_desired_m.transpose()).dot(self.W(current_quat).transpose())

        newJacobian = 0.5*block_one.dot(self.robot.bodynodes[self.bodyNodeIndex].angular_jacobian())
        newJacobian = self.selectionMatrix.dot(newJacobian)

        angular_jacobian = self.robot.bodynodes[self.bodyNodeIndex].angular_jacobian()
        angular_jacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].angular_jacobian_deriv()
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))


        #error = self.quat_desired_m.dot(current_quat_con)
        #error = (self.quat_desired_m.transpose()).dot(current_quat)
        error = self.quat_desired_m.dot(temp_id.dot(current_quat))
        #error_setpoint = np.reshape([1.0, 0.0, 0.0, 0.0],(4,1))

        #error = self.Q(current_quat_con).dot(self.desiredOrientation)
        #error = np.reshape(error - error_setpoint, (4,1))
        error = np.reshape(error, (4, 1))

        #print "The current error is: ", '\n', error.transpose()
        # test_quat = pydart.utils.transformations.quaternion_from_matrix(rotation)
        #
        # temp_id = -np.identity(4)
        # temp_id[0,0] = 1
        #
        # temp_error = (self.quat_desired_m.dot(temp_id)).dot(test_quat)
        #
        # print "The inferred  error is: ", '\n', temp_error.transpose()

        #print "The desired quaternion is: ", '\n', self.desiredOrientation
        #print "The current quaternion is: ", '\n', current_quat.transpose()
        #print "The desired quat matrix is: ", '\n',self.quat_desired_m
        #print "The temp_id matrix is: ", '\n', temp_id
        #print "The orientation task error is: ", '\n', error.transpose()

        error = self.selectionMatrix.dot(error)

        self.error[0] = error[1]
        self.error[1] = error[2]
        self.error[2] = error[3]


        constant = (0.5*block_one.dot(angular_jacobian_dot + self.Kd*angular_jacobian)).dot(dq) + self.Kp*(error)

        # dimension_Matrix = np.zeros([3,4])
        # dimension_Matrix[:, 1:4] = np.identity(3)
        #
        # newJacobian = dimension_Matrix.dot(newJacobian)
        # constant = dimension_Matrix.dot(constant)

        dimension_Matrix = np.identity(4)
        dimension_Matrix[0,0] = self.q_scalar_weight

        newJacobian = dimension_Matrix.dot(newJacobian)
        constant = dimension_Matrix.dot(constant)

        Q = newJacobian.T.dot(newJacobian)
        Q = np.block([
            [Q,          np.zeros((self.robot.ndofs, self.robot.ndofs))],
            [np.zeros((self.robot.ndofs, self.robot.ndofs)), np.zeros((self.robot.ndofs, self.robot.ndofs))]
        ])
        P = 2 * constant.T.dot(newJacobian)
        zero_vector = np.zeros((1, self.robot.ndofs))
        P = np.concatenate((P, zero_vector), axis=1)

        C = constant.T.dot(constant)

        # Q = np.identity(6)
        # P = np.zeros(( 1, 6))
        # C = 0
        if(useContactVariables):
            QP_size = 2*self.robot.ndofs
            contact_size = qpContact.Nc
            Q_new  = np.zeros((QP_size + contact_size, QP_size + contact_size))
            Q_new[:QP_size, :QP_size] = Q # write the old info
            Q = Q_new

            P_new = np.zeros((1, QP_size + contact_size))
            P_new[0, :QP_size] = P
            P = P_new

        return [self.taskWeight*Q, self.taskWeight*P, self.taskWeight*C]


if __name__ == "__main__":

    print('Hello, PyDART!')

    pydart.init()
    world_file = "../data/skel/two_cubes.skel"
    robot_file = "../data/KR5/KR5_sixx_R650.urdf"

    test_world = cubes_KR_R650.cubeKR5World_admittance_task( world_file, robot_file)

    test_robot = test_world.skeletons[-1]

    transform = test_robot.bodynodes[-1].world_transform()
    rotation = transform[:3, :3]
    current_quat = pydart.utils.transformations.quaternion_from_matrix(rotation)


    # test_desiredPosition = array([0.1, 0.2, 0.3]).reshape((3,1))
    taskWeight = 1000

    test_task = orientationTask(test_robot, current_quat, taskWeight, Kd=5, Kp=10, bodyNodeIndex=-1)

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


