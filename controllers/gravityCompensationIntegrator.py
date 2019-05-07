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

class GravityCompensationIntegrator(object):

    def __init__(self, robot, exampleController):
        self.robot = robot
        self.g = self.robot.world.gravity()
        self.enabled = True
        self.exampleController = exampleController

    def compute(self, ):

        example_acc = self.exampleController.solution

        dt = self.robot.world.dt

        example_dq = np.reshape(self.robot.dq, (self.robot.ndofs, 1)) + example_acc*dt
        example_q = np.reshape(self.robot.q, (self.robot.ndofs, 1)) + example_dq*dt

        example_dq = example_dq.flatten()
        example_q = example_q.flatten()

        self.robot.set_velocities(example_dq)
        self.robot.set_positions(example_q)

        tau = np.zeros(self.robot.num_dofs())
        if not self.enabled:
            return tau

        for body in self.robot.bodynodes:
            m = body.mass()  # Or, simply body.m
            if(len(body.dependent_dofs) is not 0 ):
                J = body.linear_jacobian(body.local_com())
                # Each time we calculate all the torque that may affect the mass.
                tau += J.transpose().dot(-(m * self.g))
        return tau