import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array


class impulseTorqueLimitConstraints:

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

        
        def calcMatricies(self):
            zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))

            G = np.concatenate((self.robot.M.dot(np.identity(self.dof)), self.robot.M.dot(-np.identity(self.dof))), axis=0)
            #G = np.concatenate((np.identity(self.dof), -np.identity(self.dof)), axis=0)
            G = np.concatenate((zero_block, G*(1/self.robot.world.dt)), axis=1)

            [h_upp, h_lower] = self.rhsVectors()

            return [G, np.concatenate((h_upp, h_lower), axis=0)]
    
