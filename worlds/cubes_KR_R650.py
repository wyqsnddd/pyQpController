import pydart2 as pydart
import numpy as np
import logging
from subprocess import call
import os

class cubeKR5World(pydart.World):

    def __init__(self, ):
        pydart.World.__init__(self, 0.005,"./data/skel/two_cubes.skel")

        logger = logging.getLogger(__name__)
        self.set_gravity([0.0, -9.81, 0.0])

        logger.info('pydart create_world OK')
        #self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650.urdf")
        self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650_palm.urdf")
        # self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650_beam.urdf")

        logger.info('pydart add_skeleton OK')

        self.jvUpper = []
        for ii in range(0, self.robot.ndofs):
            dof = self.robot.dof(ii)
            self.jvUpper.append(dof.velocity_upper_limit())

        self.tau_upper = np.ones((self.robot.ndofs, 1))*25
        self.impulse_upper = 200
        self.impulse_force_record = np.zeros((self.robot.ndofs, 1))

        self.jv_unitLength = 75
        self.tau_unitLength = 75
        self.impulse_unitLength = 75

        self.jv_pos = [80, 300]
        self.tau_pos = [250, 300]
        self.constraint_force_pos = [400, 300]

        self.impulse_pos = [900, 300]
        self.predict_jv_pos = [900, 500]

        self.f_max = np.zeros((3,1))

    def print_text(self, txt=None):
        print("Print: " + str(txt))

    def on_mouse_press(self, pos):
        print("pos = " + str(pos))
    def produce_movie(self):
        filename = "./data/captures/capture%04d.png"
        #log_file_name = os.path.join('log', 'impact_one_' + time_now + '.log')

        return_code = call("ls - lt | tail - 1", shell=True)
        start_number = os.popen("ls -t | tail -1 | egrep -o [0-9]+").read()
        start_number = start_number.strip('\n')

        amount = os.popen("find . -type f | wc -l").read()
        amount = amount.strip('\n')

        command = "ffmpeg -f image2 -r 30 -start_number " + start_number + " -i ./data/captures/robot.%4d.png -t  " + amount + " -vcodec  mpeg4 -y movie.mp4"

        print "the command is: ",'\n', command
        os.popen(command)


    def render_with_ri(self, ri):
        #ri.render_sphere(self.controller.qp.obj.tasks[0].desiredPosition, 0.05)
        ri.render_axes([0, 0, 0], 0.2, True)

        #desired_translation = self.controller.qp.obj.tasks[0].desiredPosition
        # size = [0.01, 0.01, 0.01]
        # ri.render_box(desired_translation, size)
        #ri.render_sphere(desired_translation, 0.02)
        ri.set_color(0, 0, 1)
        scale = 0.1

        p0 = self.controller.qp.contact.getContactPosition()

        K = self.controller.qp.contact.getContactGenerationMatrix()
        K_1 = K[:3, 0].reshape((3, 1))
        K_2 = K[:3, 1].reshape((3, 1))
        K_3 = K[:3, 2].reshape((3, 1))
        K_4 = K[:3, 3].reshape((3, 1))

        p1 = (p0 + K_1*scale)
        p2 = (p0 + K_2*scale)
        p3 = (p0 + K_3*scale)
        p4 = (p0 + K_4*scale)

        p0.reshape(3)

        #ri.render_axes(p0.reshape(3), 0.15, True)

        ri.set_color(0.0, 0.5, 0.5, 0.5)

        # render the friction cone:
        tau = self.robot.constraint_forces()
        J = np.reshape(self.robot.bodynodes[-1].linear_jacobian(), (3, self.robot.ndofs))
        force = np.linalg.pinv(J.transpose()).dot(tau)

        if (np.linalg.norm(force) > np.linalg.norm(self.f_max)):
            self.f_max = force



        if np.linalg.norm(self.f_max) >0 :
            ri.render_arrow(p0.reshape(3), p1.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
            ri.render_arrow(p0.reshape(3), p2.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
            ri.render_arrow(p0.reshape(3), p3.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
            ri.render_arrow(p0.reshape(3), p4.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)

            scale2 = 0.001
            # p5 = p0 + self.controller.qp.obj.tasks[0].desiredForce*scale2
            p6 = p0 - np.reshape(self.f_max, (3,1))*scale2

#            ri.set_color(1.0, 0.0, 0.0, 1.0)
            # render the desired force:
            # ri.render_arrow(p0.reshape(3), p5.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
            # # render the current force:
            ri.set_color(0.0, 1.0, 0.0, 1.0)
            ri.render_arrow(p0.reshape(3), p6.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)

         # p0 = self.controller.qp.contact.getContactPosition()
        #
        #
        #
        # K = self.controller.qp.contact.getContactGenerationMatrix()
        # K_1 = K[:3, 0].reshape((3, 1))
        # K_2 = K[:3, 1].reshape((3, 1))
        # K_3 = K[:3, 2].reshape((3, 1))
        # K_4 = K[:3, 3].reshape((3, 1))
        #
        # p1 = (p0 + K_1*scale)
        # p2 = (p0 + K_2*scale)
        # p3 = (p0 + K_3*scale)
        # p4 = (p0 + K_4*scale)
        #
        # p0.reshape(3)
        #
        # #ri.render_axes(p0.reshape(3), 0.15, True)
        #
        # ri.set_color(0.0, 0.5, 0.5, 0.5)
        #
        # ri.render_arrow(p0.reshape(3), p1.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
        # ri.render_arrow(p0.reshape(3), p2.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
        # ri.render_arrow(p0.reshape(3), p3.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
        # ri.render_arrow(p0.reshape(3), p4.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
        # # transform = self.robot.bodynodes[-1].world_transform()
        # robot_ee_translation = transform[:3, 3]
        #
        #
        # ri.render_axes(robot_ee_translation, 0.2, True)

        #ri.render_sphere([0.1, 0.1, 0.1], 1.0)
    def draw_with_ri(self, ri):
        #ri.set_color(0, 0, 1)
        #ri.render_sphere(self.controller.qp.obj.tasks[0].desiredPosition, 1.0)

        ri.set_color(0, 0, 0)

        ri.draw_text([20, 40], "time = %.4fs" % self.t)




        # ri.draw_text([20, 70], "Controller = %s" %
        #              ("ON" if self.controller.enabled else "OFF"))

        J = np.reshape(self.robot.bodynodes[-1].linear_jacobian(), (3, self.robot.ndofs))

        velocity = J.dot(np.reshape(self.skeletons[-1].dq, (self.robot.ndofs, 1)))

        ri.draw_text([300, 60], "Robot end-effector velocity = %.2f %.2f %.2f " % (
        velocity[0],
        velocity[1],
        velocity[2]))
        ri.draw_text([300, 30], "Robot weight = %.2f Kilos" % self.robot.world.skeletons[-1].mass())

        if np.linalg.norm(self.f_max) >0 :
            ri.draw_text([20, 100], "Impulse Force = %.3f %.3f %.3f " %(-self.f_max[0], -self.f_max[1], -self.f_max[2]))


        ri.set_color(0, 0, 0)

        ri.draw_text([20, 40], "time = %.4fs" %self.t )

        ri.draw_text([900, 60], "Wall friction coefficient = %.2f" %self.robot.world.skeletons[1].bodynodes[0].friction_coeff())
        ri.draw_text([900, 90], "Wall restitution coefficient = %.2f" %self.robot.world.skeletons[1].bodynodes[0].restitution_coeff())
        ri.draw_text([900, 120], "Robot palm restitution coefficient = %.2f" %self.robot.world.skeletons[-1].bodynode("palm").restitution_coeff())



        ri.draw_text([self.jv_pos[0] - self.jv_unitLength*2/3, self.jv_pos[1] - 20], "Joint velocities:")
        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.jv_pos[0] -60, self.jv_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.jv_unitLength, 20, [self.jv_pos[0], self.jv_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.robot.dq[ii])/self.jvUpper[ii])*self.jv_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.jv_unitLength - currentLength)/2
            ri.draw_box(currentLength, 20, [self.jv_pos[0] - dis, self.jv_pos[1] + ii * 30], fill=True)

        ri.draw_text([self.tau_pos[0] - self.tau_unitLength*2/3, self.tau_pos[1] - 20], "Motor torques:")
        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.tau_pos[0] -60, self.tau_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.tau_unitLength, 20, [self.tau_pos[0], self.tau_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.controller.tau_last[ii])/self.tau_upper[ii])*self.tau_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.tau_unitLength - currentLength)/2
            ri.draw_box(currentLength, 20, [self.tau_pos[0] - dis, self.tau_pos[1] + ii * 30], fill=True)

        ri.draw_text([self.constraint_force_pos[0] - self.impulse_unitLength*2/3, self.constraint_force_pos[1] - 20], "Impulse torques:")

        for ii in range(0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.constraint_force_pos[0] - 60, self.constraint_force_pos[1] + ii * 32], "%i" % ii)
            ri.draw_box(self.impulse_unitLength, 20, [self.constraint_force_pos[0], self.constraint_force_pos[1] + ii * 30], fill=False)
            # This is the current value

            #constraint_tau = abs(self.controller.jointVelocityJumpEstimator.impulse_tau[ii])
            F = self.robot.constraint_forces()
            constraint_tau = abs(F[ii])

            if constraint_tau > self.impulse_force_record[ii]:
                self.impulse_force_record[ii] = constraint_tau

            currentForceLength = (abs(self.impulse_force_record[ii]) / self.impulse_upper) * self.impulse_unitLength

            dis = (self.impulse_unitLength - currentForceLength) / 2
            current_force_origin = [self.constraint_force_pos[0] - dis, self.constraint_force_pos[1] + ii * 30]
            ri.draw_box(currentForceLength, 21, current_force_origin, fill=True)

        ri.draw_text(
            [self.impulse_pos[0] - self.impulse_unitLength * 2 / 3, self.impulse_pos[1] - 20],
            "Predicted impulse torques:")

        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.impulse_pos[0] -60, self.impulse_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.impulse_unitLength, 20, [self.impulse_pos[0], self.impulse_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.controller.jointVelocityJumpEstimator.predict_impulse_tau[ii])/self.impulse_upper)*self.impulse_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.impulse_unitLength - currentLength)/2
            current_box_origin = [self.impulse_pos[0] - dis, self.impulse_pos[1] + ii * 30]
            ri.draw_box(currentLength, 20, current_box_origin, fill=True)

        ri.draw_text(
            [self.predict_jv_pos[0] - self.jv_unitLength * 2 / 3, self.predict_jv_pos[1] - 20],
            "Predicted joint velocities after impact:")

        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.predict_jv_pos[0] -60, self.predict_jv_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.jv_unitLength, 20, [self.predict_jv_pos[0], self.predict_jv_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.controller.jointVelocityJumpEstimator.temp_v_upper[ii])/self.jvUpper[ii])*self.jv_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.jv_unitLength - currentLength)/2
            current_box_origin = [self.predict_jv_pos[0] - dis, self.predict_jv_pos[1] + ii * 30]
            ri.draw_box(currentLength+0.01, 20, current_box_origin, fill=True)


    def clear_captures(self):
        command = "rm ./data/captures/robot*"
        os.popen(command)

    def on_key_press(self, key):
        if key == 'G':
            logger = logging.getLogger(__name__)
            logger.info('Toggle gravity controller ...')
            self.controller.enabled = not self.controller.enabled

class slidingBoxKR5World(pydart.World):

    def __init__(self, ):
        pydart.World.__init__(self, 0.005,"./data/skel/sliding_box.skel")

        logger = logging.getLogger(__name__)
        self.set_gravity([0.0, -9.81, 0.0])

        logger.info('pydart create_world OK')
        #self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650.urdf")
        self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650_palm.urdf")

        logger.info('pydart add_skeleton OK')


        self.jvUpper = []
        for ii in range(0, self.robot.ndofs):
            dof = self.robot.dof(ii)
            self.jvUpper.append(dof.velocity_upper_limit())

        self.tau_upper = np.ones((self.robot.ndofs, 1))*25
        self.impulse_upper = 200
        self.impulse_force_record = np.zeros((self.robot.ndofs, 1))

        self.jv_unitLength = 75
        self.tau_unitLength = 75
        self.impulse_unitLength = 75

        self.jv_pos = [80, 300]
        self.tau_pos = [250, 300]
        self.constraint_force_pos = [400, 300]

        self.impulse_pos = [80, 500]
        self.predict_jv_pos = [400, 500]

        self.f_max = np.zeros((3, 1))

    def print_text(self, txt=None):
        print("Print: " + str(txt))

    def on_mouse_press(self, pos):
        print("pos = " + str(pos))
    def produce_movie(self):
        filename = "./data/captures/capture%04d.png"
        #log_file_name = os.path.join('log', 'impact_one_' + time_now + '.log')

        return_code = call("ls - lt | tail - 1", shell=True)
        start_number = os.popen("ls -t | tail -1 | egrep -o [0-9]+").read()
        start_number = start_number.strip('\n')

        amount = os.popen("find . -type f | wc -l").read()
        amount = amount.strip('\n')

        command = "ffmpeg -f image2 -r 30 -start_number " + start_number + " -i ./data/captures/robot.%4d.png -t  " + amount + " -vcodec  mpeg4 -y movie.mp4"

        print "the command is: ",'\n', command
        os.popen(command)


    def render_with_ri(self, ri):
        #ri.render_sphere(self.controller.qp.obj.tasks[0].desiredPosition, 0.05)

        # ri.render_axes([0, 0, 0], 0.2, True)

        transform = self.robot.bodynodes[-1].world_transform()
        robot_ee_translation = transform[:3, 3]

        #desired_translation = self.controller.qp.obj.tasks[0].desiredPosition

        #ri.render_axes(desired_translation.transpose(), 0.1, True)
        size = [0.01, 0.01, 0.01]
        #ri.render_box(desired_translation, size)
        #ri.render_sphere(desired_translation, 0.02)

        #
        #
        ri.render_axes(robot_ee_translation, 0.1, True)

        #ri.render_sphere([0.1, 0.1, 0.1], 1.0)
    def draw_with_ri(self, ri):
        #ri.set_color(0, 0, 1)
        #ri.render_sphere(self.controller.qp.obj.tasks[0].desiredPosition, 1.0)

        ri.set_color(0, 0, 0)

        ri.draw_text([20, 40], "time = %.4fs" %self.t )
        ri.draw_text([20, 60], "Controller = %s" %
                     ("Admittance " if self.robot.controller.switchedTasks else "End-effector Velocity"))
        if(self.robot.controller.switchedTasks):

            ri.draw_text([20, 40], "time = %.4fs" % self.t)
            ri.draw_text([20, 80],  "Desired Force = %.3f %.3f %.3f " %(self.robot.controller.qp.obj.tasks[0].desiredForce[0], self.robot.controller.qp.obj.tasks[0].desiredForce[1], self.robot.controller.qp.obj.tasks[0].desiredForce[2]))
            ri.draw_text([20, 100], "Current Force = %.3f %.3f %.3f " %(self.robot.controller.qp.obj.tasks[0].equivalentForceVector[0], self.robot.controller.qp.obj.tasks[0].equivalentForceVector[1], self.robot.controller.qp.obj.tasks[0].equivalentForceVector[2]))
            #ri.draw_text([20, 120], "box positions = %.3f %.3f %.3f" %(self.skeletons[2].bodynodes[0].C[0], self.skeletons[2].bodynodes[0].C[1], self.skeletons[2].bodynodes[0].C[2]))

        if self.skeletons[2].bodynodes[0].C[0]> 0.55:
            self.reset()


        J = np.reshape(self.robot.bodynodes[-1].linear_jacobian(), (3, self.robot.ndofs))

        velocity = J.dot(np.reshape(self.skeletons[-1].dq, (self.robot.ndofs, 1)))

        ri.draw_text([400, 60], "Robot end-effector velocity = %.2f %.2f %.2f " % (
        velocity[0],
        velocity[1],
        velocity[2]))
        ri.draw_text([400, 30], "Robot weight = %.2f Kilos" % self.robot.world.skeletons[-1].mass())
        ri.draw_text([800, 30], "box weight = %.2f Kilos" %self.robot.world.skeletons[2].mass())
        ri.draw_text([800, 60], "box bottom friction coefficient = %.2f" %self.robot.world.skeletons[2].bodynodes[0].friction_coeff())
        ri.draw_text([800, 90],
                      "box side friction coefficient = %.2f" % self.robot.world.skeletons[2].bodynodes[1].friction_coeff())

        ri.draw_text([800, 120],
                      "box side restitution coefficient = %.2f" % self.robot.world.skeletons[2].bodynodes[1].restitution_coeff())



        ri.draw_text([self.jv_pos[0] - self.jv_unitLength*2/3, self.jv_pos[1] - 20], "Joint velocities:")
        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.jv_pos[0] -60, self.jv_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.jv_unitLength, 20, [self.jv_pos[0], self.jv_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.robot.dq[ii])/self.jvUpper[ii])*self.jv_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.jv_unitLength - currentLength)/2
            ri.draw_box(currentLength, 20, [self.jv_pos[0] - dis, self.jv_pos[1] + ii * 30], fill=True)

        ri.draw_text([self.tau_pos[0] - self.tau_unitLength*2/3, self.tau_pos[1] - 20], "Motor torques:")
        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.tau_pos[0] -60, self.tau_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.tau_unitLength, 20, [self.tau_pos[0], self.tau_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.controller.tau_last[ii])/self.tau_upper[ii])*self.tau_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.tau_unitLength - currentLength)/2
            ri.draw_box(currentLength, 20, [self.tau_pos[0] - dis, self.tau_pos[1] + ii * 30], fill=True)

        ri.draw_text([self.constraint_force_pos[0] - self.impulse_unitLength*2/3, self.constraint_force_pos[1] - 20], "Impulse torques:")

        for ii in range(0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.constraint_force_pos[0] - 60, self.constraint_force_pos[1] + ii * 32], "%i" % ii)
            ri.draw_box(self.impulse_unitLength, 20, [self.constraint_force_pos[0], self.constraint_force_pos[1] + ii * 30], fill=False)
            # This is the current value

            #constraint_tau = abs(self.controller.jointVelocityJumpEstimator.impulse_tau[ii])
            F = self.robot.constraint_forces()
            constraint_tau = abs(F[ii])

            if constraint_tau > self.impulse_force_record[ii]:
                self.impulse_force_record[ii] = constraint_tau

            currentForceLength = (abs(self.impulse_force_record[ii]) / self.impulse_upper) * self.impulse_unitLength

            dis = (self.impulse_unitLength - currentForceLength) / 2
            current_force_origin = [self.constraint_force_pos[0] - dis, self.constraint_force_pos[1] + ii * 30]
            ri.draw_box(currentForceLength, 21, current_force_origin, fill=True)

        ri.draw_text(
            [self.impulse_pos[0] - self.impulse_unitLength * 2 / 3, self.impulse_pos[1] - 20],
            "Predicted impulse torques:")

        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.impulse_pos[0] -60, self.impulse_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.impulse_unitLength, 20, [self.impulse_pos[0], self.impulse_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.controller.jointVelocityJumpEstimator.predict_impulse_tau[ii])/self.impulse_upper)*self.impulse_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.impulse_unitLength - currentLength)/2
            current_box_origin = [self.impulse_pos[0] - dis, self.impulse_pos[1] + ii * 30]
            ri.draw_box(currentLength, 20, current_box_origin, fill=True)

        ri.draw_text(
            [self.predict_jv_pos[0] - self.jv_unitLength * 2 / 3, self.predict_jv_pos[1] - 20],
            "Predicted joint velocities after impact:")

        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.predict_jv_pos[0] -60, self.predict_jv_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.jv_unitLength, 20, [self.predict_jv_pos[0], self.predict_jv_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.controller.jointVelocityJumpEstimator.temp_v_upper[ii])/self.jvUpper[ii])*self.jv_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.jv_unitLength - currentLength)/2
            current_box_origin = [self.predict_jv_pos[0] - dis, self.predict_jv_pos[1] + ii * 30]
            ri.draw_box(currentLength+0.01, 20, current_box_origin, fill=True)


    def set_big_box_friction(self, coe):
        self.robot.world.skeletons[2].bodynodes[0].set_friction_coeff(coe)

    def set_side_box_friction(self, coe):
        self.robot.world.skeletons[2].bodynodes[1].set_friction_coeff(coe)

    def set_side_box_resis(self, coe):
        self.robot.world.skeletons[2].bodynodes[1].set_restitution_coeff(coe)

    def set_big_box_mass(self, mass):
        self.robot.world.skeletons[2].bodynodes[0].set_mass(mass)
        print "The side box weight is: ", self.robot.world.skeletons[2].bodynodes[0].mass()

    def set_side_box_mass(self, mass):
        self.robot.world.skeletons[2].bodynodes[1].set_mass(mass)
        print "The side box weight is: ",self.robot.world.skeletons[2].bodynodes[1].mass()


    def reset_velocity_task(self, velocity):
        if (self.robot.controller.switchedTasks):
            print "The transation velocity task is already removed"
        else:
            self.robot.controller.qp.obj.tasks[0].desiredTranslationVelocity[0]=velocity
            print "The desired translation velocity is: ",self.robot.controller.qp.obj.tasks[0].desiredTranslationVelocity[0]


    def reset_admittance_task(self, force=None):
        if(self.robot.controller.switchedTasks):
            self.robot.controller.qp.obj.tasks[0].desiredForce[0] = force
            print "The desired admittance task force is: ", self.robot.controller.qp.obj.tasks[0].desiredForce[0]
        else:
            print "The admittance task is not activated yet"

    def clear_captures(self):
        command = "rm ./data/captures/robot*"
        os.popen(command)



    def on_key_press(self, key):
        if key == 'G':
            logger = logging.getLogger(__name__)
            logger.info('Toggle gravity controller ...')
            self.controller.enabled = not self.controller.enabled




class cubeTwoKR5World(pydart.World):

    def __init__(self, ):
        pydart.World.__init__(self, 0.005,"./data/skel/three_cubes.skel")

        logger = logging.getLogger(__name__)
        self.set_gravity([0.0, -9.81, 0.0])

        logger.info('pydart create_world OK')

        self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650.urdf")

        self.robot_two = self.add_skeleton("./data/KR5/KR5_sixx_R650_offset.urdf")
        #self.robot_two = self.add_skeleton("./data/KR5/KR5_sixx_R650_offset_collision_gravity_free.urdf")

        logger.info('pydart add two skeletons OK')

    def print_text(self, txt=None):
        print("Print: " + str(txt))

    def on_mouse_press(self, pos):
        print("pos = " + str(pos))
    def clear_captures(self):
        command = "rm ./data/captures/robot*"
        os.popen(command)

    def produce_movie(self):
        filename = "./data/captures/capture%04d.png"
        #log_file_name = os.path.join('log', 'impact_one_' + time_now + '.log')

        return_code = call("ls - lt | tail - 1", shell=True)
        start_number = os.popen("ls -t | tail -1 | egrep -o [0-9]+").read()
        start_number = start_number.strip('\n')

        amount = os.popen("find . -type f | wc -l").read()
        amount = amount.strip('\n')

        command = "ffmpeg -f image2 -r 30 -start_number " + start_number + " -i ./data/captures/robot.%4d.png -t  " + amount + " -vcodec  mpeg4 -y movie.mp4"

        print "the command is: ",'\n', command
        os.popen(command)


    def render_with_ri(self, ri):
        #ri.render_sphere(self.controller.qp.obj.tasks[0].desiredPosition, 0.05)
        ri.render_axes([0, 0, 0], 0.2, True)

        #desired_translation = self.controller.qp.obj.tasks[0].desiredPosition
        # size = [0.01, 0.01, 0.01]
        # ri.render_box(desired_translation, size)
        #ri.render_sphere(desired_translation, 0.02)

        # transform = self.robot.bodynodes[-1].world_transform()
        # robot_ee_translation = transform[:3, 3]
        #
        #
        # ri.render_axes(robot_ee_translation, 0.2, True)

        #ri.render_sphere([0.1, 0.1, 0.1], 1.0)
    def draw_with_ri(self, ri):
        #ri.set_color(0, 0, 1)
        #ri.render_sphere(self.controller.qp.obj.tasks[0].desiredPosition, 1.0)

        ri.set_color(0, 0, 0)

        ri.draw_text([20, 40], "time = %.4fs" % self.t)
        ri.draw_text([20, 70], "Controller = %s" %
                     ("ON" if self.controller.enabled else "OFF"))
        ri.draw_text([638, 427], "QP + DART")
        ri.draw_text([607, 556], "QP + Integrator + gravity compensation")

    def on_key_press(self, key):
        if key == 'G':
            logger = logging.getLogger(__name__)
            logger.info('Toggle gravity controller ...')
            self.controller.enabled = not self.controller.enabled

class cubeKR5World_trajectory_task(pydart.World):

    def __init__(self, ):
        pydart.World.__init__(self, 0.005,"./data/skel/two_cubes.skel")

        logger = logging.getLogger(__name__)
        self.set_gravity([0.0, -9.81, 0.0])

        logger.info('pydart create_world OK')
        self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650.urdf")

        logger.info('pydart add_skeleton OK')

    def print_text(self, txt=None):
        print("Print: " + str(txt))

    def on_mouse_press(self, pos):
        print("mouse screen pos = " + str(pos))

    def render_with_ri(self, ri):
        ri.set_color(0, 0, 1)
        n = self.controller.qp.obj.tasks[0].controlPoints.shape[1]
        for ii in range(0, n):
            translation = self.controller.qp.obj.tasks[0].controlPoints[:,ii]
            ri.render_sphere(translation, 0.01)

        ri.set_color(1, 0, 0)

        desired_translation = self.controller.qp.obj.tasks[0].desiredPosition
        desired_velocity = self.controller.qp.obj.tasks[0].desiredVelocity

        #size = [0.01, 0.01, 0.01]
        #ri.render_box(desired_translation, size)

        ri.render_axes([0, 0, 0], 0.2, True)

        # transform = self.robot.bodynodes[-1].world_transform()
        # robot_ee_translation = transform[:3, 3]
        #
        # ri.render_axes(robot_ee_translation, 0.2, True)

        ri.render_sphere(desired_translation, 0.02)

        # render desired velocity
        ri.render_sphere(desired_translation, 0.02)

        desired_velocity_render = (robot_ee_translation).reshape((3, 1)) + desired_velocity
        #desired_velocity_render = robot_ee_translation + (desired_velocity).reshape((1,3))
        #p0 = [robot_ee_translation[0], robot_ee_translation[1], robot_ee_translation[2]]
        p0 = []
        p0.append(desired_translation.item(0))
        p0.append(desired_translation.item(1))
        p0.append(desired_translation.item(2))

        p1 = [desired_velocity_render[0], desired_velocity_render[1], desired_velocity_render[2]]
        p1 =[]
        p1.append(desired_velocity_render.item(0))
        p1.append(desired_velocity_render.item(1))
        p1.append(desired_velocity_render.item(2))

        ri.render_arrow(p0, p1)

        #ri.render_arrow(robot_ee_translation, desired_velocity_render)
            #ri.render_arrow(robot_ee_translation, desired_velocity_render, 0.05, 0.1, 0.1)

        #ri.render_sphere([0.1, 0.1, 0.1], 1.0)
    def draw_with_ri(self, ri):
        ri.set_color(0, 0, 1)

        # We render all the way points on the trajectory
        #ri.render_sphere(self.controller.qp.obj.tasks[0].desiredPosition, 1.0)

        ri.set_color(0, 0, 0)

        ri.draw_text([20, 40], "time = %.4fs" % self.t)
        ri.draw_text([20, 70], "Controller = %s" %
                     ("ON" if self.controller.enabled else "OFF"))

    def on_key_press(self, key):
        #print "capture key: ", key
        if key == 'G':
            logger = logging.getLogger(__name__)
            logger.info('Toggle gravity controller ...')
            self.controller.enabled = not self.controller.enabled

class cubeKR5World_admittance_task(pydart.World):

    def __init__(self, world_file=None, robot_file=None):
        if world_file == None:
            pydart.World.__init__(self, 0.005,"./data/skel/two_cubes.skel")
        else:
            pydart.World.__init__(self, 0.005, world_file)

        logger = logging.getLogger(__name__)
        self.set_gravity([0.0, -9.81, 0.0])
        #self.set_gravity([0.0, 0.0, 0.0])

        logger.info('pydart create_world OK')
        if robot_file == None:
            #self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650.urdf")
            self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650_palm.urdf")

        else:
            self.robot = self.add_skeleton(robot_file)

        logger.info('pydart add_skeleton OK')

        self.force = None
        self.duration = 0


        self.jvUpper = []
        for ii in range(0, self.robot.ndofs):
            dof = self.robot.dof(ii)
            self.jvUpper.append(dof.velocity_upper_limit())

        self.tau_upper = np.ones((self.robot.ndofs, 1))*25
        self.impulse_upper = 200
        self.impulse_force_record = np.zeros((self.robot.ndofs, 1))

        self.jv_unitLength = 75
        self.tau_unitLength = 75
        self.impulse_unitLength = 75

        self.jv_pos = [80, 300]
        self.tau_pos = [250, 300]
        self.constraint_force_pos = [400, 300]

        self.impulse_pos = [900, 300]
        self.predict_jv_pos = [900, 500]

        self.f_max = np.zeros((3, 1))


    def on_key_press(self, key):
        if key == 'G':
            logger = logging.getLogger(__name__)
            logger.info('Toggle gravity controller ...')
            self.controller.enabled = not self.controller.enabled

    def on_step_event(self, ):
        print "duration is: ", self.duration
        if self.force is not None and self.duration >= 0:
            self.duration -= 1
            #self.skeletons[-1].body('h_spine').add_ext_force(self.force)
            self.skeletons[-1].body('palm').add_ext_force(self.force)
    def produce_movie(self):
        filename = "./data/captures/capture%04d.png"
        #log_file_name = os.path.join('log', 'impact_one_' + time_now + '.log')

        return_code = call("ls - lt | tail - 1", shell=True)
        start_number = os.popen("ls -t | tail -1 | egrep -o [0-9]+").read()
        start_number = start_number.strip('\n')

        amount = os.popen("find . -type f | wc -l").read()
        amount = amount.strip('\n')

        command = "ffmpeg -f image2 -r 30 -start_number " + start_number + " -i ./data/captures/robot.%4d.png -t  " + amount + " -vcodec  mpeg4 -y movie.mp4"

        print "the command is: ",'\n', command
        os.popen(command)

    def print_text(self, txt=None):
        print("Print: " + str(txt))

    def calc_different_torque(self):

        result = np.reshape(self.robot.forces(), (6, 1)) - np.reshape(self.robot.body('palm').linear_jacobian(), (6, 3)).dot(np.reshape((self.force), (3, 1)))
        print ("The torque difference is: ", result)

    def on_mouse_press(self, pos):
        print("Mouse screen pos = " + str(pos))


    def render_with_ri(self, ri):
        ri.set_color(0, 0, 1)
        scale = 0.1

        #if(self.robot.controller.switchedTasks):
        #self.render_palm_contact(ri)

        p0 = self.controller.qp.contact.getContactPosition()

        K = self.controller.qp.contact.getContactGenerationMatrix()
        K_1 = K[:3, 0].reshape((3, 1))
        K_2 = K[:3, 1].reshape((3, 1))
        K_3 = K[:3, 2].reshape((3, 1))
        K_4 = K[:3, 3].reshape((3, 1))

        p1 = (p0 + K_1*scale)
        p2 = (p0 + K_2*scale)
        p3 = (p0 + K_3*scale)
        p4 = (p0 + K_4*scale)

        p0.reshape(3)

        #ri.render_axes(p0.reshape(3), 0.15, True)

        ri.set_color(0.0, 0.5, 0.5, 0.5)

        # render the friction cone:

        if self.controller.qp.getContactStatus():
            ri.render_arrow(p0.reshape(3), p1.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
            ri.render_arrow(p0.reshape(3), p2.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
            ri.render_arrow(p0.reshape(3), p3.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
            ri.render_arrow(p0.reshape(3), p4.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)

            scale2 = 0.01
            p5 = p0 + self.controller.qp.obj.tasks[0].desiredForce*scale2
            p6 = p0 + self.controller.qp.obj.tasks[0].currentForce*scale2

            ri.set_color(1.0, 0.0, 0.0, 1.0)
            # render the desired force:
            ri.render_arrow(p0.reshape(3), p5.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)
            # render the current force:
            ri.set_color(0.0, 1.0, 0.0, 1.0)
            ri.render_arrow(p0.reshape(3), p6.reshape(3), r_base=0.003, head_width=0.015, head_len=0.01)

        if self.force is not None and self.duration >= 0:
            p0 = self.skeletons[-1].body('palm').C
            print "p0 is: ", p0
            p1 = p0 + 0.01*self.force
            ri.set_color(0.0, 1.0, 0.0)
            ri.render_arrow(p0.reshape(3), p1.reshape(3), r_base=0.05, head_width=0.1, head_len=0.1)


        # n = self.controller.qp.obj.tasks[0].controlPoints.shape[1]
        # for ii in range(0, n):
        #     translation = self.controller.qp.obj.tasks[0].controlPoints[:,ii]
        #     ri.render_sphere(translation, 0.01)
        #
        # ri.set_color(1, 0, 0)
        #
        # desired_translation = self.controller.qp.obj.tasks[0].desiredPosition
        # #size = [0.01, 0.01, 0.01]
        # #ri.render_box(desired_translation, size)
        # ri.render_sphere(desired_translation, 0.02)

        ri.render_axes([0, 0, 0], 0.1, True)

        #transform = self.robot.bodynodes[-1].world_transform()
        #robot_ee_translation = transform[:3, 3]

        #ri.render_axes(robot_ee_translation, 0.2, True)

        #ri.render_sphere([0.1, 0.1, 0.1], 1.0)



    def draw_with_ri(self, ri):
        ri.set_color(0, 0, 1)

        # We render all the way points on the trajectory
        #ri.render_sphere(self.controller.qp.obj.tasks[0].desiredPosition, 1.0)

        ri.set_color(0, 0, 0)

        ri.draw_text([20, 40], "time = %.4fs" % self.t)

        ri.draw_text([20, 60], "Controller = %s" %
                     ("Admittance " if self.robot.controller.switchedTasks else "End-effector Velocity"))


        if(self.robot.controller.switchedTasks):
            ri.draw_text([20, 40], "time = %.4fs" % self.t)
            ri.draw_text([20, 80],  "Desired Force = %.3f %.3f %.3f " %(self.robot.controller.qp.obj.tasks[0].desiredForce[0], self.robot.controller.qp.obj.tasks[0].desiredForce[1], self.robot.controller.qp.obj.tasks[0].desiredForce[2]))
            ri.draw_text([20, 100], "Current Force = %.3f %.3f %.3f " %(self.robot.controller.qp.obj.tasks[0].equivalentForceVector[0], self.robot.controller.qp.obj.tasks[0].equivalentForceVector[1], self.robot.controller.qp.obj.tasks[0].equivalentForceVector[2]))

        # render the friction cone:
        tau = self.robot.constraint_forces()
        J = np.reshape(self.robot.bodynodes[-1].linear_jacobian(), (3, self.robot.ndofs))
        force = np.linalg.pinv(J.transpose()).dot(tau)

        if (np.linalg.norm(force) > np.linalg.norm(self.f_max)):
            self.f_max = force

        if np.linalg.norm(self.f_max) >0 :
            ri.draw_text([20, 120], "Impulse Force = %.3f %.3f %.3f " %(-self.f_max[0], -self.f_max[1], -self.f_max[2]))

        J = np.reshape(self.robot.bodynodes[-1].linear_jacobian(), (3, self.robot.ndofs))

        velocity = J.dot(np.reshape(self.skeletons[-1].dq, (self.robot.ndofs, 1)))

        ri.draw_text([350, 60], "Robot end-effector velocity = %.2f %.2f %.2f " % (
        velocity[0],
        velocity[1],
        velocity[2]))
        ri.draw_text([350, 30], "Robot weight = %.2f Kilos" % self.robot.world.skeletons[-1].mass())

        ri.set_color(0, 0, 0)

        ri.draw_text([20, 40], "time = %.4fs" %self.t )

        ri.draw_text([900, 60], "Wall friction coefficient = %.2f" %self.robot.world.skeletons[1].bodynodes[0].friction_coeff())
        ri.draw_text([900, 90], "Wall restitution coefficient = %.2f" %self.robot.world.skeletons[1].bodynodes[0].restitution_coeff())
        ri.draw_text([900, 120], "Robot palm restitution coefficient = %.2f" %self.robot.world.skeletons[-1].bodynode("palm").restitution_coeff())



        ri.draw_text([self.jv_pos[0] - self.jv_unitLength*2/3, self.jv_pos[1] - 20], "Joint velocities:")
        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.jv_pos[0] -60, self.jv_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.jv_unitLength, 20, [self.jv_pos[0], self.jv_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.robot.dq[ii])/self.jvUpper[ii])*self.jv_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.jv_unitLength - currentLength)/2
            ri.draw_box(currentLength, 20, [self.jv_pos[0] - dis, self.jv_pos[1] + ii * 30], fill=True)

        ri.draw_text([self.tau_pos[0] - self.tau_unitLength*2/3, self.tau_pos[1] - 20], "Motor torques:")
        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.tau_pos[0] -60, self.tau_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.tau_unitLength, 20, [self.tau_pos[0], self.tau_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.controller.tau_last[ii])/self.tau_upper[ii])*self.tau_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.tau_unitLength - currentLength)/2
            ri.draw_box(currentLength, 20, [self.tau_pos[0] - dis, self.tau_pos[1] + ii * 30], fill=True)

        ri.draw_text([self.constraint_force_pos[0] - self.impulse_unitLength*2/3, self.constraint_force_pos[1] - 20], "Impulse torques:")

        for ii in range(0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.constraint_force_pos[0] - 60, self.constraint_force_pos[1] + ii * 32], "%i" % ii)
            ri.draw_box(self.impulse_unitLength, 20, [self.constraint_force_pos[0], self.constraint_force_pos[1] + ii * 30], fill=False)
            # This is the current value

            #constraint_tau = abs(self.controller.jointVelocityJumpEstimator.impulse_tau[ii])
            F = self.robot.constraint_forces()
            constraint_tau = abs(F[ii])

            if constraint_tau > self.impulse_force_record[ii]:
                self.impulse_force_record[ii] = constraint_tau

            currentForceLength = (abs(self.impulse_force_record[ii]) / self.impulse_upper) * self.impulse_unitLength

            dis = (self.impulse_unitLength - currentForceLength) / 2
            current_force_origin = [self.constraint_force_pos[0] - dis, self.constraint_force_pos[1] + ii * 30]
            ri.draw_box(currentForceLength, 21, current_force_origin, fill=True)

        ri.draw_text(
            [self.impulse_pos[0] - self.impulse_unitLength * 2 / 3, self.impulse_pos[1] - 20],
            "Predicted impulse torques:")

        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.impulse_pos[0] -60, self.impulse_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.impulse_unitLength, 20, [self.impulse_pos[0], self.impulse_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.controller.jointVelocityJumpEstimator.predict_impulse_tau[ii])/self.impulse_upper)*self.impulse_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.impulse_unitLength - currentLength)/2
            current_box_origin = [self.impulse_pos[0] - dis, self.impulse_pos[1] + ii * 30]
            ri.draw_box(currentLength, 20, current_box_origin, fill=True)

        ri.draw_text(
            [self.predict_jv_pos[0] - self.jv_unitLength * 2 / 3, self.predict_jv_pos[1] - 20],
            "Predicted joint velocities after impact:")

        for ii in range (0, self.robot.ndofs):
            # This is the unit length
            ri.draw_text([self.predict_jv_pos[0] -60, self.predict_jv_pos[1] + ii*32], "%i"%ii)
            ri.draw_box(self.jv_unitLength, 20, [self.predict_jv_pos[0], self.predict_jv_pos[1] + ii*30], fill=False)
            # This is the current value
            currentLength = (abs(self.controller.jointVelocityJumpEstimator.temp_v_upper[ii])/self.jvUpper[ii])*self.jv_unitLength
            #ri.draw_box(currentLength, 20, [50 - 30, 50 + ii*30], fill=True)
            dis = (self.jv_unitLength - currentLength)/2
            current_box_origin = [self.predict_jv_pos[0] - dis, self.predict_jv_pos[1] + ii * 30]
            ri.draw_box(currentLength+0.01, 20, current_box_origin, fill=True)


            
    def clear_captures(self):
        command = "rm ./data/captures/robot*"
        os.popen(command)
