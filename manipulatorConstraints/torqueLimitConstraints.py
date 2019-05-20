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
from cvxopt import normal, uniform
from numpy import array


class torqueLimitConstraints:
    """!@brief
            @class torqueLimitConstraints
            defined for acc variables
            We generate right hand side 'B' for constraints in the form: Ax <= B
    """


    def __init__(self, robot, impactRobust=False, upper=None, lower=None):
        self.robot = robot
        self.dof = self.robot.ndofs

        if upper is None:
            self.torqueUpper = self.robot.tau_upper.reshape((self.dof, 1))
        else:
            if upper.shape == (self.dof,1):
                self.torqueUpper = upper
            else:
                raise Exception("upper size does not match")

        if lower is None:
            self.torqueLower = self.robot.tau_lower.reshape((self.dof, 1))
        else:
            if lower.shape == (self.dof, 1):
                self.torqueLower = lower
            else:
                raise Exception("lower size does not match")

        self.impactRobust = impactRobust


    def rhsVectors(self):

        eeJacobian_T = self.robot.bodynodes[-1].world_jacobian().transpose()

        invM = np.linalg.inv(self.robot.M)

        N_C = self.robot.coriolis_and_gravity_forces()

        if(self.impactRobust):
            upperRhs = (self.torqueUpper - np.reshape(N_C, (self.dof, 1)) )
            lowerRhs = -(self.torqueLower - np.reshape(N_C, (self.dof, 1)) )
        else:
            F = self.robot.constraint_forces()
            upperRhs = (self.torqueUpper - np.reshape(N_C, (self.dof, 1))  + np.reshape(F, (self.dof, 1)) )
            lowerRhs = -(self.torqueLower - np.reshape(N_C, (self.dof, 1)) + np.reshape(F, (self.dof, 1)))

        
        upperRhs = np.reshape(upperRhs, (self.dof, 1))

                
        lowerRhs = np.reshape(lowerRhs, (self.dof, 1))

        return [upperRhs, lowerRhs]
    def update(self, impactEstimator):
        pass

    def calcMatricies(self, useContactVariables, qpContact):
        zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))

        G = np.concatenate((self.robot.M.dot(np.identity(self.dof)), self.robot.M.dot(-np.identity(self.dof))), axis=0)
        G = np.concatenate((G, zero_block), axis=1)

        [h_upp, h_lower] = self.rhsVectors()

        if (useContactVariables):
            tempElement = qpContact.getJacobianTranspose().dot(qpContact.getContactGenerationMatrix())
            G_con = np.concatenate(
                (- tempElement,
                 tempElement,
                 ),
                axis=0)
            G = np.concatenate((G, G_con), axis=1)

        return [G, np.concatenate((h_upp, h_lower), axis=0)]
   
