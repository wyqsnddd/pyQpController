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