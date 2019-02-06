##
# @file sliding_box_admittance_impact.py
#

import pydart2 as pydart
import numpy as np


from controllers import manipulatorImpactController


from utils import print_skeleton

from worlds import cubes_KR_R650

import json
import time
import os
import logging
import logging.config

import signal
import sys

from functools import partial

def signal_handler(world, signal, frame):

    print('Hello, I think you pressed Ctrl+C!')

    world.controller.logData()

    sys.exit(0)



if __name__ == '__main__':
    print('Example: QP impact controller for manipulator')



    pydart.init()
    print('pydart initialization OK')

    #logging.config.fileConfig('logging.ini')

    time_now = time.strftime("%b_%d_%Y_%H-%M-%S", time.gmtime())



    log_file_name = os.path.join('log', 'impact_one_' + time_now +'.log')
    pydart.utils.log.setup(log_file_name)

    root = logging.getLogger()
    root.setLevel(logging.INFO)

    print "logger is set."

    world = cubes_KR_R650.slidingBoxKR5World()

    print('pydart create_world OK')

    with open('config/sliding_box_admittance_impact.json') as file:
        qpData = json.load(file)
    # while world.t < 2.0:
    #     if world.nframes % 100 == 0:
    #         skel = world.skeletons[-1]
    #         print("%.4fs: The last cube COM = %s" % (world.t, str(skel.C)))
    #     world.step()

    skel= world.skeletons[-1]


    restitutionCoeff = qpData["simulationWorldParameters"]["box_surface_restitution_coeff"]
    boxMass = qpData["simulationWorldParameters"]["box_weight"]
    
    # world.skeletons[1].bodynodes[0].set_friction_coeff(frictionCoeff)
    # world.skeletons[1].bodynodes[0].set_restitution_coeff(restitutionCoeff)
    world.skeletons[2].bodynodes[1].set_restitution_coeff(restitutionCoeff)
    world.skeletons[2].bodynodes[0].set_mass(boxMass)

    print_skeleton.skeleton_printer(skel)


    #skel.set_controller(gravityCompensationController.GravityCompensationController(skel))
    #test_desiredPosition = np.array([0.1, 0.2, 0.3]).reshape((3, 1))
    world.controller = manipulatorImpactController.manipulatorImpactController(skel, qpData, world.dt)
    skel.set_controller(world.controller)

    # set the friction coefficient of the floating box
    world.skeletons[2].bodynodes[0].set_friction_coeff(world.controller.box_bottom_friction_coeff)
    world.skeletons[2].bodynodes[1].set_friction_coeff(world.controller.box_coarse_friction_coeff)



    print('Create controller OK')

    signal.signal(signal.SIGINT, partial(signal_handler, world))
    print('Registered SIGINT Handler OK')

    # self.controller =

    # Use a zero disturbance acc controller
    # target  = np.zeros(self.robot.num_dofs())
    # self.controller = executeACC.jointAccController(self.robot, target)

    #win = pydart.gui.viewer.launch(world,"QP Impact Controller", default_camera=0)  # Use Z-up camera
    win = pydart.gui.pyqt5.window.PyQt5Window(world, "QP_IMPACT_controller")

    # win = pydart.gui.viewer.PydartWindow(world)
    win.camera_event(0)
    win.set_capture_rate(10)



    panel = pydart.gui.pyqt5.side_panel.SidePanel()
    panel.add_label("--------------------------Movie----------------------------")
    panel.add_push_button("Clear Captures", world.clear_captures, next_line=False)
    panel.add_push_button("Record Movie", world.produce_movie)
    #panel.add_combobox("Render", ["No", "Yes"], default=1, callback=world.print_text, label="MyRenderSetting")

    panel.add_label("-----------Friction and Impact----------------------")
    panel.add_double_spinbox("Coarse side friction Coe", 0, 1.0, step=0.05, callback=world.set_side_box_friction)
    panel.add_double_spinbox("Coarse side resistitution Coe", 0, 1.0, step=0.05, callback=world.set_side_box_resis)
    panel.add_double_spinbox("Bottom side friction Coe", 0, 1.0, step=0.05, callback=world.set_big_box_friction)

    panel.add_label("-------------------------------Mass------------------------")
    panel.add_double_spinbox("Box mass", 0.01, 1000.0, step=2, callback=world.set_big_box_mass)
    panel.add_double_spinbox("Side box mass", 0.001, 10.0, step=0.01, callback=world.set_side_box_mass)

    panel.add_label("------------------------Admittance task ------------------")
    panel.add_double_spinbox("Desired force along the normal", 0, 1000.0, step=1.0, callback=world.reset_admittance_task)

    panel.add_label("------------------------Velocity task ----------------------")
    panel.add_double_spinbox("Desired velocity along the normal", 0, 10.0, step=0.01, callback=world.reset_velocity_task)

    win.add_right_panel(panel)
    win.run_application()
