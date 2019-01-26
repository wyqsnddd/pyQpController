import pydart2 as pydart
import math
import numpy as np
from cvxopt import normal, uniform
from numpy import array
from worlds import cubes_KR_R650
from controllers import gravityCompensationController
from qpControllers import qpObj

import logging


class qpContact:
    """!@brief
    This class keeps and updates contact location and outputs friction cone generating matrix, whose columns are the generating vectors.
    It has two status, (1): keeps fixed contact J\dot{q} = 0; (2) regulate contact force along the normal direction of the surface
    It uses different axis to select between mode (1) and (2)
    """
    def __init__(self, skel, frictionCoe=0.7, Nc=4):
        self.robot = skel

        # number of generating vectors
        self.Nc = Nc

        self.frictionCoe = frictionCoe

        self.update()

        # Normal direction of the contact surface
        #self.surfaceNormal

        # Position of the contact location: in this minimal example, we use the center of the palm
        #self.contactPosition

        # Contact generating matrix
        #self.K

        # selection matrix

        # weights
        test_weights = np.zeros(Nc)
        self.weights = np.asarray(test_weights).reshape((self.Nc, 1))

        # I may need to  extract the palm center point transform.

    def getSurfaceNormal(self):
        return self.surfaceNormal

    def getContactPosition(self):
        return self.contactPosition


    def getContactGenerationMatrix(self):
        return self.K

    def updateWeights(self, input):
        self.weights = np.asarray(input).reshape((self.Nc, 1))
        # print "The f_QP force is: ", self.getContactForce().transpose()

    def getWeights(self):
        return self.weights


    def calcContactConstraintMatrices(self):
        G = np.zeros((self.Nc, 2*self.robot.ndofs + self.Nc))
        G[:, 2*self.robot.ndofs:] = -np.identity(self.Nc)
        h = np.reshape( np.zeros((self.Nc, 1)) , (self.Nc, 1))
        return [G, h]


    def getContactForce(self):
        return self.getContactGenerationMatrix().dot(self.getWeights())

    def getJacobianTranspose(self):
        return self.robot.bodynodes[-1].linear_jacobian().transpose()
    
    def update(self):

        # (0) update the contact location
        transform = self.robot.bodynodes[-1].world_transform()
        # check if this is the palm or the end-effector?

        robot_ee_translation = transform[:3, 3].reshape((3, 1))
        robot_ee_translation[0] = robot_ee_translation[0] + 0.02
        robot_ee_rotation = transform[:3, :3].reshape((3, 3))

        robot_x_axis = transform[:3, 0].reshape((3, 1))
        robot_y_axis = transform[:3, 1].reshape((3, 1))

        self.contactPosition = robot_ee_translation

        # (1) update the surface normal
        self.surfaceNormal = robot_x_axis

        # (2) update the generating matrix
        self.K = np.zeros( (3, self.Nc) )

        angle = math.atan(self.frictionCoe)

        tangentDirection = robot_y_axis

        # A vector on the cone
        tempRotation = pydart.utils.transformations.rotation_matrix(angle, tangentDirection)[:3, :3].reshape((3, 3))
        generator = tempRotation.dot(self.getSurfaceNormal())
        generator.reshape((3,1))
        angleStep = 2*math.pi/self.Nc

        for i in range(0, self.Nc):
            tempRotation = pydart.utils.transformations.rotation_matrix( i*angleStep, self.getSurfaceNormal())[:3,:3]
            self.K[:3, i] = tempRotation.dot(generator).reshape(3)













































