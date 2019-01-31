import pydart2 as pydart

import numpy as np
from cvxopt import normal, uniform
from numpy import array


class torqueLimitConstraints:

    def __init__(self, robot, impactRobust=False, upper=None, lower=None):
        self.robot = robot
        self.dof = self.robot.ndofs

        #self.torqueUpper = self.robot.tau_upper
        #self.torqueLower = self.robot.tau_upper

        # if upper.shape[1] is not 1:
        #     raise Exception("Upper limits is not a vector")
        # if lower.shape[1] is not 1:
        #     raise Exception("Lower limits is not a vector")
        # if upper.shape[0] is not self.dof:
        #     raise Exception("Upper limits is not correct")
        # if lower.shape[0] is not self.dof:
        #     raise Exception("Lower limits is not correct")

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

        self.impactRobust = impactRobust

        #print "tau_upper is : ", self.torqueUpper
        #print "tau_lower is : ", self.torqueLower

    def rhsVectors(self):

        eeJacobian_T = self.robot.bodynodes[-1].world_jacobian().transpose()

        invM = np.linalg.inv(self.robot.M)
        # print "Test: m*inv_m: ", self.robot.M*invM

        N_C = self.robot.coriolis_and_gravity_forces()
        #print "N_C is : ", N_C

        #print "F is : ", F

        # if(self.robot.controller.constraintForceAware):
        #     upperRhs = self.torqueUpper - np.reshape(N_C, (self.dof, 1)) + np.reshape(F, (self.dof, 1))
        #     lowerRhs = -(self.torqueLower - np.reshape(N_C, (self.dof, 1)) + np.reshape(F, (self.dof, 1)))
        # else:
        # upperRhs = (self.torqueUpper - np.reshape(N_C, (self.dof, 1)))
        # lowerRhs = -(self.torqueLower - np.reshape(N_C, (self.dof, 1)))
        if(self.impactRobust):
            upperRhs = (self.torqueUpper - np.reshape(N_C, (self.dof, 1)) )
            lowerRhs = -(self.torqueLower - np.reshape(N_C, (self.dof, 1)) )
        else:
            F = self.robot.constraint_forces()
            upperRhs = (self.torqueUpper - np.reshape(N_C, (self.dof, 1))  + np.reshape(F, (self.dof, 1)) )
            lowerRhs = -(self.torqueLower - np.reshape(N_C, (self.dof, 1)) + np.reshape(F, (self.dof, 1)))

        
        #upperRhs = self.torqueUpper - np.reshape(N_C, (self.dof, 1))+ np.reshape(eeJacobian_T.dot(F), (self.dof, 1))
        #upperRhs = self.torqueUpper - np.reshape(N_C, (self.dof, 1))+ np.reshape(F, (self.dof, 1))
        #upperRhs = self.torqueUpper - np.reshape(N_C, (self.dof, 1)) #+ np.reshape(F, (self.dof, 1))

        #upperRhs = invM.dot(np.reshape(upperRhs, (self.dof, 1)))
        upperRhs = np.reshape(upperRhs, (self.dof, 1))

        #lowerRhs = -(self.torqueLower - np.reshape(N_C, (self.dof, 1)) + np.reshape(eeJacobian_T.dot(F), (self.dof, 1)))
        #lowerRhs = -(self.torqueLower - np.reshape(N_C, (self.dof, 1)) + np.reshape(F, (self.dof, 1)))
        #lowerRhs = -(self.torqueLower - np.reshape(N_C, (self.dof, 1)) )# + np.reshape(F, (self.dof, 1))

        lowerRhs = np.reshape(lowerRhs, (self.dof, 1))
        #lowerRhs = invM.dot(np.reshape(lowerRhs, (self.dof, 1)))

        #test_bound = upperRhs - lowerRhs

        #print "test_bound is: ", '\n', test_bound


        return [upperRhs, lowerRhs]
    def update(self, impactEstimator):
        pass

    def calcMatricies(self, useContactVariables, qpContact):
        zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))

        G = np.concatenate((self.robot.M.dot(np.identity(self.dof)), self.robot.M.dot(-np.identity(self.dof))), axis=0)
        #G = np.concatenate((np.identity(self.dof), -np.identity(self.dof)), axis=0)
        G = np.concatenate((G, zero_block), axis=1)

        [h_upp, h_lower] = self.rhsVectors()

        if (useContactVariables):
            # Append more columns corresponding to the contact force varialbes
            # contact_size = qpContact.Nc
            # row_number = G.shape[0]
            # column_number = G.shape[1]
            # G_new = np.zeros((row_number, column_number + contact_size))
            # G_new[:, :column_number] = G  # write the old info
            #
            # G_new[:,column_number:] =
            tempElement = qpContact.getJacobianTranspose().dot(qpContact.getContactGenerationMatrix())
            G_con = np.concatenate(
                (- tempElement,
                 tempElement,
                 ),
                axis=0)
            G = np.concatenate((G, G_con), axis=1)


            # Keep h as it is.

        return [G, np.concatenate((h_upp, h_lower), axis=0)]
    
    # def calcContactMatricies(self, contact):
    #     zero_block = np.zeros((2*self.robot.ndofs, self.robot.ndofs))
    #
    #     G = np.concatenate((self.robot.M.dot(np.identity(self.dof)), self.robot.M.dot(-np.identity(self.dof))), axis=0)
    #     #G = np.concatenate((np.identity(self.dof), -np.identity(self.dof)), axis=0)
    #     G = np.concatenate((G, zero_block), axis=1)
    #
    #     # torque constraint
    #     tempElement = contact.getJacobianTranspose().dot(contact.getContactGenerationMatrix())
    #     G_con = np.concatenate(
    #         ( - tempElement,
    #           tempElement,
    #         ),
    #         axis=0)
    #     G = np.concatenate((G, G_con), axis=1)
    #
    #     [h_upp, h_lower] = self.rhsVectors()
    #
    #     return [G, np.concatenate((h_upp, h_lower), axis=0)]

if __name__ == "__main__":
    print('Hello, PyDART!')

    pydart.init()

    test_world = pydart.World( 1.0/2000.0, "../data/skel/two_cubes.skel")


    test_robot = test_world.add_skeleton("../data/KR5/KR5_sixx_R650.urdf")

    upper = 200*np.reshape(array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), (6,1))
    lower = 200*np.reshape(array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]), (6, 1))

    test_tlc = torqueLimitConstraints(test_robot, upper, lower )
    #test_tlc = torqueLimitConstraints(test_robot)


    test_rhs = test_tlc.rhsVectors()

    print "The test upper is: ", test_rhs[0]

    print "The test lower is: ", test_rhs[1]

    [G, h ] = test_tlc.calcMatricies()
    print "The G  is: ",'\n', G
    print "The h is: ",'\n', h



