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

from cvxopt import matrix, solvers

class jointAccelerationLimitConstraints:
    """!@brief
        defined for acc variables
        We generate right hand side 'B' for constraints in the form: Ax <= B
    """
    def __init__(self, skel):
        self.robot = skel

        self.upper = 12*np.array([0.87266, 0.87266, 0.87266, 1.466, 2.059, 2.094])
        self.upper = np.reshape(self.upper, (self.robot.ndofs, 1))
        self.lower = -self.upper
        
        print ("The joint acc upper limit is: ", self.upper)
        print ("The joint acc lower limit is: ", self.lower)

    def update(self, impactEstimator):
        self.average_impact_acc = impactEstimator.readAverageDdq()
        self.average_impact_acc = np.reshape(self.average_impact_acc, (self.robot.ndofs, 1))
        
    def calcMatricies(self, useContactVariables, qpContact):
        zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))

        G = np.concatenate((np.identity(self.robot.ndofs), -np.identity(self.robot.ndofs)), axis=0)

        G = np.concatenate((G, zero_block), axis=1)

        h = np.reshape(np.concatenate((self.upper, -self.lower)),(12,1))
                         
        if (useContactVariables):
            # Append more columns corresponding to the contact force varialbes
            contact_size = qpContact.Nc
            row_number = G.shape[0]
            column_number = G.shape[1]
            G_new = np.zeros((row_number, column_number + contact_size))
            G_new[:, :column_number] = G  # write the old info
            G = G_new

        return [G, h]

