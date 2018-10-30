import pydart2 as pydart
import numpy as np

from controllers import gravityCompensationIntegrator
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

    world = cubes_KR_R650.cubeTwoKR5World()

    print('pydart create_world OK')

    with open('config/two_KR5_hybrid_impact_controller_one.json') as file:
        qpData = json.load(file)
    # while world.t < 2.0:
    #     if world.nframes % 100 == 0:
    #         skel = world.skeletons[-1]
    #         print("%.4fs: The last cube COM = %s" % (world.t, str(skel.C)))
    #     world.step()

    skel= world.skeletons[-2]

    print_skeleton.skeleton_printer(skel)


    #skel.set_controller(gravityCompensationController.GravityCompensationController(skel))
    #test_desiredPosition = np.array([0.1, 0.2, 0.3]).reshape((3, 1))
    world.controller = manipulatorImpactController.manipulatorImpactController(skel, qpData, world.dt)
    skel.set_controller(world.controller)


    print('Create controller OK')

    robot_qp = world.skeletons[-1]
    robot_qp.set_controller(gravityCompensationIntegrator.GravityCompensationIntegrator(robot_qp, world.controller))

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
    panel.add_label("Mode")
    panel.add_push_button("Clear Captures", world.clear_captures, next_line=False)
    panel.add_push_button("Record Movie", world.produce_movie)
    panel.add_combobox("Render", ["No", "Yes"], default=1, callback=world.print_text, label="MyRenderSetting")
    panel.add_double_spinbox("Step", 0, 10.0, step=2.5, callback=world.print_text)
    panel.set_text("Mode", "Melo")

    win.add_right_panel(panel)
    win.run_application()
