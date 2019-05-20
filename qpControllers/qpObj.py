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

import logging

from manipulatorTasks import positionTask


class qpObj:
    """!@brief
        @class qpObj
        Based on the impact dynamics model, we use the joint accelerations( to be decided by the QP in the CURRENT step) to predict the joint velocity jumps for the NEXT step
        We generate 'A' and 'B' matricies for constraints in the form: Ax <= B
    """
    ## The constructor
    def __init__(self, skel, jointUnitWeight, deltaDqWeight, contactWeight, qpContact):
        """!@brief The constructor."""

        self.tasks = []
        self.robot = skel
        self.dof = self.robot.ndofs
        #self.logger = logger


        self.dofWeightMatrix = np.identity(2*self.dof)
        self.contactDofWeightMatrix = np.identity(2 * self.dof + qpContact.Nc)


        # set the weights of the joint, descend from base to end
        for ii in range(0,self.dof):
            #self.dofWeightMatrix[ii,ii] = (self.dof - ii)*jointUnitWeight
            self.dofWeightMatrix[ii, ii] = jointUnitWeight
            self.contactDofWeightMatrix[ii, ii] = jointUnitWeight
            
        for ii in range(self.dof, 2*self.dof):
            # We need to maximize the delta QP 
            # self.dofWeightMatrix[ii, ii] = -jointUnitWeight
            self.dofWeightMatrix[ii, ii] = deltaDqWeight
            self.contactDofWeightMatrix[ii, ii] = deltaDqWeight

        for ii in range(2*self.dof, 2*self.dof + qpContact.Nc):
            self.contactDofWeightMatrix[ii, ii] = contactWeight



    def numTasks(self):
        return len(self.tasks)

    def weightMatrix(self):
        return self.dofWeightMatrix

    def addTask(self, task):
        self.tasks.append(task)

    def removeTask(self, task):
        """!@brief Remove the task.
        @param task to be removed
        """
        self.tasks.remove(task)

    def calcMatricies(self, useContactVariables, qpContact):

        if useContactVariables:
            qpSize = 2*self.dof + qpContact.Nc
            dofWeightMatrix = self.contactDofWeightMatrix
        else:
            qpSize = 2*self.dof
            dofWeightMatrix = self.dofWeightMatrix

        Q = np.zeros((qpSize, qpSize))
        P = np.zeros(( 1, qpSize))
        C = 0.0

        Q += dofWeightMatrix

        for ii in range(0, len(self.tasks)):
            [Q_add, P_add, C_add ] = self.tasks[ii].calcMatricies(useContactVariables, qpContact)
            Q += Q_add
            P += P_add
            C += C_add

        return [Q, P, C]





