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

class robustJointVelocityLimitConstraints:
    """!@brief
    @class robustJointVelocityLimitConstraints
    We use Euler forward to approximate the derivative and introduce the impact robust joint velocity constraints. 
    """

    def __init__(self, skel, dt):
        self.robot = skel
        if ((dt<0.0) and (dt > 1.0)):
            raise Exception("Unproper dt")

        self.upper = []
        self.lower = []
        for ii in range(0, self.robot.ndofs):
            dof = self.robot.dof(ii)
            self.upper.append(dof.velocity_upper_limit())
            self.lower.append(dof.velocity_lower_limit())

        print ("The joint velocity upper limit is: ", self.upper)
        print ("The joint velocity lower limit is: ", self.lower)
        
        self.upper = np.reshape(self.upper, (self.robot.ndofs, 1))
        self.lower = np.reshape(self.lower, (self.robot.ndofs, 1))

        self.dt = dt

    def upperRhs(self, dq):
        return  (self.upper - dq)

    def update(self, impactEstimator):
        pass

    def lowerRhs(self, dq):
        return    -(self.lower - dq)


    def calcMatricies(self, useContactVariables, qpContact):
        zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))

        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        G = np.concatenate((np.identity(self.robot.ndofs), -np.identity(self.robot.ndofs)), axis=0)
        G = np.concatenate((zero_block, G), axis=1)
        h = np.concatenate(( self.upperRhs(dq), self.lowerRhs(dq)), axis=0)

        if (useContactVariables):
            # Append more columns corresponding to the contact force varialbes
            contact_size = qpContact.Nc
            row_number = G.shape[0]
            column_number = G.shape[1]
            G_new = np.zeros((row_number, column_number + contact_size))
            G_new[:, :column_number] = G  # write the old info
            G = G_new
            # Keep h as it is.

        return [G, h]
