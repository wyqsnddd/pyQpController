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
        # self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650.urdf")
        self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650_palm.urdf")
        #self.robot = self.add_skeleton("./data/KR5/KR5_sixx_R650_beam.urdf")

        logger.info('pydart add_skeleton OK')

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




        # ri.draw_text([20, 70], "Controller = %s" %
        #              ("ON" if self.controller.enabled else "OFF"))

        J = np.reshape(self.robot.bodynodes[-1].linear_jacobian(), (3, self.robot.ndofs))

        velocity = J.dot(np.reshape(self.skeletons[-1].dq, (self.robot.ndofs, 1)))

        ri.draw_text([300, 60], "Robot end-effector velocity = %.2f %.2f %.2f " % (
        velocity[0],
        velocity[1],
        velocity[2]))
        ri.draw_text([300, 30], "Robot weight = %.2f Kilos" % self.robot.world.skeletons[-1].mass())

        ri.set_color(0, 0, 0)

        ri.draw_text([20, 40], "time = %.4fs" %self.t )

        ri.draw_text([900, 60], "Wall friction coefficient = %.2f" %self.robot.world.skeletons[1].bodynodes[0].friction_coeff())
        ri.draw_text([900, 90], "Wall restitution coefficient = %.2f" %self.robot.world.skeletons[1].bodynodes[0].restitution_coeff())
        ri.draw_text([900, 120], "Robot palm restitution coefficient = %.2f" %self.robot.world.skeletons[-1].bodynode("palm").restitution_coeff())

        

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

        desired_translation = self.controller.qp.obj.tasks[0].desiredPosition

        #ri.render_axes(desired_translation.transpose(), 0.1, True)
        size = [0.01, 0.01, 0.01]
        ri.render_box(desired_translation, size)
        ri.render_sphere(desired_translation, 0.02)

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

        if self.force is not None and self.duration >= 0:
            p0 = self.skeletons[-1].body('palm').C
            p1 = p0 + 0.01 * self.force
            ri.set_color(1.0, 0.0, 0.0)
            ri.render_arrow(p0, p1, r_base=0.05, head_width=0.1, head_len=0.1)

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

        ri.render_axes([0, 0, 0], 0.2, True)

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

            
    def clear_captures(self):
        command = "rm ./data/captures/robot*"
        os.popen(command)
