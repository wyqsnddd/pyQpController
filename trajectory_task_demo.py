import pydart2 as pydart
import numpy as np

from controllers import gravityCompensationController
from controllers import manipulatorController


from utils import print_skeleton

from worlds import cubes_KR_R650

import json
import time
import os
import logging
import logging.config

if __name__ == '__main__':
    print('Example: QP impact controller for manipulator')



    pydart.init(verbose=True)
    print('pydart initialization OK')

    #logging.config.fileConfig('logging.ini')

    time_now = time.strftime("%b_%d_%Y_%H-%M-%S", time.gmtime())
    log_file_name = os.path.join('log', 'impact_one_' + time_now +'.log')
    pydart.utils.log.setup(log_file_name)

    logger = logging.getLogger()
    logger.setLevel(logging.WARNING)

    print "logger is set."

    world = cubes_KR_R650.cubeKR5World_trajectory_task()

    logger.info('pydart create_world OK')

    with open('config/trajectory_task_one.json') as file:
        qpData = json.load(file)

    skel= world.skeletons[-1]

    print_skeleton.skeleton_printer(skel)

    world.controller = manipulatorController.manipulatorController(skel, qpData, world.dt)
    skel.set_controller(world.controller)

    logger.info('Create controller OK')

    #win = pydart.gui.viewer.launch(world,"QP Impact Controller", default_camera=0)  # Use Z-up camera
    win = pydart.gui.pyqt5.window.PyQt5Window(world, "QP_IMPACT_controller")

    #win = pydart.gui.viewer.PydartWindow(world)
    win.camera_event(0)
    win.set_capture_rate(10)



    panel = pydart.gui.pyqt5.side_panel.SidePanel()
    panel.add_label("Mode")
    panel.add_push_button("Print One", world.print_text, next_line=False)
    panel.add_push_button("Print Two", world.print_text)
    panel.add_combobox("Render", ["No", "Yes"], default=1, callback=world.print_text, label="MyRenderSetting")
    panel.add_double_spinbox("Step", 0, 10.0, step=2.5, callback=world.print_text)
    panel.set_text("Mode", "Melo")

    win.add_right_panel(panel)
    win.run_application()
