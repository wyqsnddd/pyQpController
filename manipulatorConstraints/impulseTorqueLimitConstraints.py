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


class impulseTorqueLimitConstraints:
        """!@brief
                @class impulseTorqueLimitConstraints
                defined for acc variables
                We generate right hand side 'B' for constraints in the form: Ax <= B
        """

        def __init__(self, robot, upper=None, lower=None):
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
            
        def update(self, impactEstimator):
            pass

        def rhsVectors(self):

            upperRhs = self.torqueUpper
            lowerRhs = -self.torqueLower

            upperRhs = np.reshape(upperRhs, (self.dof, 1))

            lowerRhs = np.reshape(lowerRhs, (self.dof, 1))
                
            return [upperRhs, lowerRhs]

        
        def calcMatricies(self, useContactVariables, qpContact):
            zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))
            # instead of Mass matrix we need to use the weighted Jacobian pseudo inverse
            jacobian = self.robot.bodynodes[-1].linear_jacobian()

            M_inv = np.linalg.pinv(self.robot.M)
            temp = np.linalg.pinv(jacobian.dot(M_inv).dot(jacobian.transpose()) )
            J_dagger = jacobian.transpose().dot(temp)

            component = J_dagger.dot(jacobian)
            G = np.concatenate((component.dot(np.identity(self.dof)), component.dot(-np.identity(self.dof))),
                                axis=0)

            G = np.concatenate((zero_block, G*(1/self.robot.world.dt)), axis=1)

            [h_upp, h_lower] = self.rhsVectors()

            if (useContactVariables):
                # Append more columns corresponding to the contact force varialbes
                contact_size = qpContact.Nc
                row_number = G.shape[0]
                column_number = G.shape[1]
                G_new = np.zeros((row_number, column_number + contact_size))
                G_new[:, :column_number] = G  # write the old info
                G = G_new
                # Keep h as it is.

            return [G, np.concatenate((h_upp, h_lower), axis=0)]
    
