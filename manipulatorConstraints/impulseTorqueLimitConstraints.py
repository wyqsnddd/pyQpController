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

        
        def calcMatricies(self, useContactVariables, qpContact):
            zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))
            # instead of Mass matrix we need to use the weighted Jacobian pseudo inverse
            jacobian = self.robot.bodynodes[-1].linear_jacobian()

            M_inv = np.linalg.pinv(self.robot.M)
            temp = np.linalg.pinv(jacobian.dot(M_inv).dot(jacobian.transpose()) )
            J_dagger = jacobian.transpose().dot(temp)

            # G = np.concatenate((J_dagger.dot(np.identity(self.dof)), J_dagger.dot(-np.identity(self.dof))),
            #                    axis=0)

            # G = np.concatenate((J_dagger.dot(np.identity(self.dof)), J_dagger.dot(-np.identity(self.dof))),
            #                    axis=0)


            component = J_dagger.dot(jacobian)
            G = np.concatenate((component.dot(np.identity(self.dof)), component.dot(-np.identity(self.dof))),
                                axis=0)

            # G = np.concatenate((self.robot.M.dot(np.identity(self.dof)), self.robot.M.dot(-np.identity(self.dof))), axis=0)
            
            #G = np.concatenate((np.identity(self.dof), -np.identity(self.dof)), axis=0)

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
    
