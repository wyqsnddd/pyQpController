import pydart2 as pydart
import numpy as np

class GravityCompensationController(object):

    def __init__(self, robot):
        self.robot = robot
        self.g = self.robot.world.gravity()
        self.enabled = True

    def compute(self, ):
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