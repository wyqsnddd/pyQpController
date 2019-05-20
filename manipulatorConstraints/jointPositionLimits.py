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

class jointLimitConstraints:
    """!@brief
        @class jointLimitConstraints
        defined for acc variables
        We generate right hand side 'B' for constraints in the form: Ax <= B
    """

    def __init__(self, skel, dt):
        self.robot = skel

        if ((dt<0.0) and (dt > 1.0)):
            raise Exception("Unproper dt")

        self.upper = (self.robot.position_upper_limits()).reshape((self.robot.ndofs, 1))
        self.lower = (self.robot.position_lower_limits()).reshape((self.robot.ndofs, 1))
        print ("The joint upper limit is: ", self.upper.transpose())
        print ("The joint lower limit is: ", self.lower.transpose())

        self.dt = dt

    def upperRhs(self, q, dq):
        return (self.upper - q)*(1/(self.dt*self.dt)) - dq*(1/self.dt)

    def update(self, impactEstimator):
        pass

    def lowerRhs(self, q, dq):
        return -((self.lower - q)*(1/(self.dt*self.dt)) - dq*(1/self.dt))

    def calcMatricies(self, useContactVariables, qpContact):
        zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))
        
        q = (self.robot.q).reshape((self.robot.ndofs,1))
        dq = (self.robot.dq).reshape((self.robot.ndofs, 1))

        G = np.concatenate((np.identity(self.robot.ndofs), -np.identity(self.robot.ndofs)), axis=0)
        G = np.concatenate((G, zero_block), axis=1)
        h = np.concatenate(( self.upperRhs(q, dq), self.lowerRhs(q, dq)), axis=0)

        if(useContactVariables):
            # Append more columns corresponding to the contact force varialbes
            contact_size = qpContact.Nc
            row_number = G.shape[0]
            column_number = G.shape[1]
            G_new = np.zeros((row_number, column_number + contact_size))
            G_new[:, :column_number] = G  # write the old info
            G = G_new
            
            # Keep h as it is. 
            


        
        return [G, h]


