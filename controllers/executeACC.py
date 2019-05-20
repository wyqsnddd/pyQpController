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

class jointAccController:
    def __init__(self,skel,target_acc):
        self.skel = skel
        self.target = target_acc
        self.enabled = True


    def compute(self):
        tau = np.zeros(self.skel.num_dofs())
        if not self.enabled:
            return tau
        # use the dynamics equation to generate the desired torque

        tau = self.skel.M.dot(self.target) + self.skel.c - self.skel.constraint_forces()

        return tau


