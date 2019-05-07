import pydart2 as pydart
import numpy as np
import scipy.interpolate as si

from cvxopt import normal, uniform
from numpy import array
import logging
from qpControllers import  qpObj
from manipulatorTasks import positionTask

class bs_interpolator:
    def __init__(self, controlPoints, timeKnots, n, degree=1):
        """ Calculate n samples on a bspline

                controlPoints :   Array ov control vertices
                timeKnots  :      Time vector associated with the controlPoints
                n: Number of samples to return
                degree:   Curve degree
                periodic: True - Curve is closed
                          False - Curve is open
            """
        self.cv = np.asarray(controlPoints)
        count = len(self.cv)

        # Adjust the time knot vector: len(timeKnots) = degree + len(controlPoints) + 1, we choose d=2, thus add two time points to the knots
        kv = np.insert(timeKnots, 0, timeKnots[0])
        kv = np.insert(kv, -1, timeKnots[-1])
        self.kv = kv

        # kv = [timeKnots[0], timeKnots, timeKnots[-1] ]

        # Calculate query range
        self.u = np.linspace(kv[0], kv[-1], n)
        # False, (count - degree), n)
        self.degree = degree

    def calcTrajectory(self):
        return np.array(si.splev(self.u, (self.kv, self.cv.T, self.degree))).T

    def readTimeKnots(self):
        return self.u

    def interpolate(self, time):
        return np.array(si.splev(time, (self.kv, self.cv.T, self.degree))).T

    def interpolate_d(self, time, d):
        return np.array(si.splev(time, (self.kv, self.cv.T, self.degree), d)).T


class trajectoryTask(positionTask.positionTask):
    def __init__(self, skel, controlPoints, timeKnots, taskWeight, samplePointsNumber=None, Kd=None, Kp=None, bodyNodeIndex=None):

        placeholder_desiredPosition = array([0.1, 0.2, 0.3]).reshape((3, 1))

        positionTask.positionTask.__init__(self, skel, placeholder_desiredPosition, taskWeight, Kd, Kp, bodyNodeIndex)

        if (controlPoints.shape[1]!=len(timeKnots)):
            raise  Exception("The length of timeKnots and controlPoints mismatch")


        if samplePointsNumber == None:
            self.samplePointsNumber = 100
        else:
            self.samplePointsNumber = samplePointsNumber

        self.bs_interpolator = bs_interpolator(controlPoints.T, timeKnots, self.samplePointsNumber)
        self.controlPoints = controlPoints
        self.timeKnots = timeKnots
        # Use the first point of the trajectory
        input = self.bs_interpolator.interpolate(0.0)
        input_v = self.bs_interpolator.interpolate_d(0.0, 1)

        #self.desiredPosition = self.toVector(input[1:4:1])
        self.desiredPosition = self.toVector(input)
        self.desiredVelocity = self.toVector(input_v)
        self.error = np.zeros((3, 1))
        self.V_error = np.zeros((3, 1))
        #self.input_last = self.desiredPosition

    def toVector(self, input):
        return np.reshape(input, (3,1))

    def updateTargetPose(self):
        timeNow = self.robot.world.t
        #delta_t = self.robot.world.dt

        if timeNow <= self.timeKnots[-1]:
            input = self.bs_interpolator.interpolate(timeNow)
            #input_v = (input.reshape((3,1)) - self.input_last )/delta_t
            input_v = self.bs_interpolator.interpolate_d(timeNow, 1)

            #self.input_last = input.reshape((3,1))
            self.desiredPosition = self.toVector(input)
            self.desiredVelocity = self.toVector(input_v)

        #print "The desired position is: ", self.desiredPosition.T

    def update(self):
        # This needs to be called in every time step
        self.updateTargetPose()

    def calcMatricies(self):

        newJacobian = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian()

        newJacobian_dot = self.robot.bodynodes[self.bodyNodeIndex].linear_jacobian_deriv()

        transform = self.robot.bodynodes[self.bodyNodeIndex].world_transform()
        translation = transform[[0, 1, 2], 3].reshape((3, 1))
        # print "The transform is: ", transform, " shape: ", transform.shape
        # print "The translation is: ", translation

        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        self.error = (translation - self.desiredPosition)

        velocity = self.robot.bodynodes[self.bodyNodeIndex].com_linear_velocity()
        velocity = newJacobian.dot(dq)

        self.V_error = ( velocity.reshape((3,1))  - self.desiredVelocity)
        # logger = logging.getLogger(__name__)
        # logger.info('The position task error is: %s ', error)

        # print "The position task error is: ", '\n', error
        #constant = (newJacobian_dot + self.Kd * newJacobian).dot(dq) + self.Kp * error

        constant = (newJacobian_dot + self.Kd * newJacobian).dot(dq) + self.Kd * self.V_error + self.Kp * self.error
        # 2*self.Kw*newJacobian.dot(self.robot.dq) + (translation - self.desiredPosition)


        Q = newJacobian.T.dot(newJacobian)
        
        Q_size = Q.shape[0]
        Q_new  = np.zeros((Q_size + self.robot.ndofs, Q_size + self.robot.ndofs))
        Q_new[:Q_size, :Q_size] = Q

        Q = Q_new        
        # Q = np.block([
        #     [Q,          np.zeros((self.robot.ndofs, self.robot.ndofs))],
        #     [np.zeros((self.robot.ndofs, self.robot.ndofs)), np.zeros((self.robot.ndofs, self.robot.ndofs))]
        # ])

        P = 2 * constant.T.dot(newJacobian)
        zero_vector = np.zeros((1, self.robot.ndofs))
        P = np.concatenate((P, zero_vector), axis=1)

        C = constant.T.dot(constant)

        # return [newJacobian, newJacobian_dot, Q, P, C]
        return [self.taskWeight * Q, self.taskWeight * P, self.taskWeight * C]
if __name__ == "__main__":

    print('Hello, PyDART!')

    pydart.init()

    test_world = pydart.World(1.0 / 2000.0, "../data/skel/two_cubes.skel")

    test_robot = test_world.add_skeleton("../data/KR5/KR5_sixx_R650.urdf")

    cv_3 = np.array([[50., 25., 1.],
                     [59., 12., 3.],
                     [50., 10., 5.],
                     [57., 2., 8.],
                     [40., 4., 10.],
                     [40., 14., 12.]]).T

    timeKnots = [0.1, 0.2, 0.4, 0.7, 2.0, 3.0]

    #test_desiredPosition = array([0.1, 0.2, 0.3]).reshape((3,1))
    taskWeight = 1000

    test_task = trajectoryTask(test_robot, cv_3, timeKnots, taskWeight)

    [Q, P, C] = test_task.calcMatricies()

    # print "The jacobian is: ", '\n', jacobian
    # print "The jacobian derivative is: ", '\n', jacobian_dot
   #  print "The Q matrix is: ", '\n', Q
   #  print "The P matrix is: ", '\n', Pp
   #  print "The C matrix is: ", '\n', C

    test_obj = qpObj.qpObj(test_robot, 100)
    test_obj.addTask(test_task)


   # print "The weight matrix is: ", '\n', test_obj.dofWeightMatrix
   # print "The numer of tasks is: ", test_obj.numTasks()


    [Q_obj, P_obj, C_obj] = test_obj.calcMatricies()
    # print "The Q_obj matrix is: ", '\n', Q_obj
    # print "The P_obj matrix is: ", '\n', P_obj
    # print "The C_obj matrix is: ", '\n', C_obj
