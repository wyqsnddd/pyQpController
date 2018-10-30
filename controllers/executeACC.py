
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

        # Use the end-effector Jacobian
        J = self.skel.bodynodes[-1].jacobian()

        # It seems that we can work with this equality if there is no forces
        #tau = self.skel.M.dot(self.target) + self.skel.c - J.transpose().dot(self.skel.constraint_forces())
        tau = self.skel.M.dot(self.target) + self.skel.c - self.skel.constraint_forces()

        return tau


