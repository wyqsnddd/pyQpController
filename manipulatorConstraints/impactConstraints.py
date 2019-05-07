import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array


class impactConstraints:
    """!@brief
        @class impactConstraints
        Based on the impact dynamics model, we use the joint accelerations( to be decided by the QP in the CURRENT step) to predict the joint velocity jumps for the NEXT step
        We generate 'A' and 'B' matricies for constraints in the form: Ax <= B
    """
    def __init__(self, robot, res = 1.0, bodyNodeIndex=None):
        """!@brief The constructor."""
        self.robot = robot
        self.dof = self.robot.ndofs
        self.res = res
        
        if bodyNodeIndex is None:
            self.bodyNodeIndex = -1
        else:
            self.bodyNodeIndex = bodyNodeIndex


    def update(self, impactEstimator):
        """Place holder."""
        pass

    def calcMatricies(self, useContactVariables, qpContact):
        """!@brief is called in every step to construct the QP
        @param useContactVariables, if true, we fit the dims for weights of the generating matricies.
        @return the matrices A, B for the equality constraint Ax <= B
        """

        jacobian = self.robot.bodynodes[-1].linear_jacobian()
        M_inv = np.linalg.pinv(self.robot.M)
        temp = np.linalg.pinv(jacobian.dot(M_inv).dot(jacobian.transpose()) )
        J_dagger = jacobian.transpose().dot(temp)

        dq =  (self.robot.dq).reshape((self.robot.ndofs, 1))

        temp_A = (self.res + 1)*(M_inv.dot(J_dagger)).dot(jacobian)

        # A = np.block([
        #     [temp_A*self.robot.world.dt, np.identity(self.robot.ndofs)]
        # ])

        A_new  = np.zeros((temp_A.shape[0], temp_A.shape[1] + self.robot.ndofs))
        A_new[:temp_A.shape[0], :temp_A.shape[1]] = temp_A*self.robot.world.dt
        A_new[:temp_A.shape[0], temp_A.shape[1]:] = np.identity(self.robot.ndofs)

        A = A_new
        
        b  = -temp_A.dot(dq)


        if(useContactVariables):
            # Append more columns corresponding to the contact force varialbes
            contact_size = qpContact.Nc
            row_number = A.shape[0]
            column_number = A.shape[1]
            A_new = np.zeros((row_number, column_number + contact_size))
            A_new[:, :column_number] = A  # write the old info
            A = A_new

        
        return [A, b]
