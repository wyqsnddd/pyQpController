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
from contact import qpContact

class contactEqualityConstraint:
    # We use this equality constaint to link two optimization varialbes: (1)the joint accelerations, (2) The weights of the contact friction cone generators. 

    def __init__(self, robot, input_qpContact):
        self.robot = robot
        self.dof = self.robot.ndofs
        self.contact = input_qpContact
        
    def rhsVectors(self):

        # tau = np.reshape(self.robot.tau(), (self.dof, 1))
        N_C = np.reshape(self.robot.coriolis_and_gravity_forces(), (self.dof, 1))
        return - N_C   

    
    def update(self, impactEstimator):
        pass


    def calcMatricies(self):
        A_ddq = self.robot.M
        eeJacobian_T = self.robot.bodynodes[-1].linear_jacobian().transpose()

        A_lambda = - eeJacobian_T.dot(self.contact.getContactGenerationMatrix())
        A_torque = - np.identity(self.robot.ndofs)

        return [A_ddq, A_torque, A_lambda, self.rhsVectors()]


