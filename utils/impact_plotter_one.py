import sys
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties


if __name__ =="__main__":
    fileName =  sys.argv[1]
    loaded = np.load(fileName)
    #loaded = np.load("../log/data/data_Sep_28_2018_10-39-46.npz")
    #loaded = np.load("../log/data/data_Oct_23_2018_21-43-14.npz")

    time = loaded['time']

    error_x = loaded['error'][:, 0]
    error_y = loaded['error'][:, 1]
    error_z = loaded['error'][:, 2]


    fontP = FontProperties()
    fontP.set_size('small')



    # fig1, (ax11, ax12, ax13 )= plt.subplots(nrows=3, ncols=1)
    # ax1 = plt.subplot(311)
    # plt.plot(time, error_x, label='error x')
    # ax11.set_ylabel('error x')
    #
    # ax12 = plt.subplot(312)
    # plt.plot(time, error_y, label='error y')
    # ax12.set_ylabel('error y')
    #
    # ax13 = plt.subplot(313)
    # plt.plot(time, error_z, label='error z')
    # plt.ylabel('error z')
    # plt.grid(True)
    # plt.xlabel('Time [s]')
    #
    fig12 = plt.figure()

    ax = fig12.gca()
    ax.plot(time, error_x, label='Error x')
    ax.plot(time, error_y, label='Error y')
    ax.plot(time, error_z, label='Error z')
    ax.legend(frameon=False, loc='upper left', prop=fontP)
    plt.xlabel('Time [s]')
    plt.ylabel('Task Error')
    plt.grid(True)
    fig12.savefig("task_error.pdf", bbox_inches='tight')

    q_0 = loaded['q'][:, 0]
    q_1 = loaded['q'][:, 1]
    q_2 = loaded['q'][:, 2]
    q_3 = loaded['q'][:, 3]
    q_4 = loaded['q'][:, 4]
    q_5 = loaded['q'][:, 5]

    dq_0 = loaded['dq'][:, 0]
    dq_1 = loaded['dq'][:, 1]
    dq_2 = loaded['dq'][:, 2]
    dq_3 = loaded['dq'][:, 3]
    dq_4 = loaded['dq'][:, 4]
    dq_5 = loaded['dq'][:, 5]

    acc_0 = loaded['acc'][:, 0]
    acc_1 = loaded['acc'][:, 1]
    acc_2 = loaded['acc'][:, 2]
    acc_3 = loaded['acc'][:, 3]
    acc_4 = loaded['acc'][:, 4]
    acc_5 = loaded['acc'][:, 5]

    tau_0 = loaded['tau'][:, 0]
    tau_1 = loaded['tau'][:, 1]
    tau_2 = loaded['tau'][:, 2]
    tau_3 = loaded['tau'][:, 3]
    tau_4 = loaded['tau'][:, 4]
    tau_5 = loaded['tau'][:, 5]

    rc_0 = loaded['robot_c'][:, 0]
    rc_1 = loaded['robot_c'][:, 1]
    rc_2 = loaded['robot_c'][:, 2]
    rc_3 = loaded['robot_c'][:, 3]
    rc_4 = loaded['robot_c'][:, 4]
    rc_5 = loaded['robot_c'][:, 5]

    sol_q_0 = loaded['sol_q'][:, 0]
    sol_q_1 = loaded['sol_q'][:, 1]
    sol_q_2 = loaded['sol_q'][:, 2]
    sol_q_3 = loaded['sol_q'][:, 3]
    sol_q_4 = loaded['sol_q'][:, 4]
    sol_q_5 = loaded['sol_q'][:, 5]

    sol_dq_0 = loaded['sol_dq'][:, 0]
    sol_dq_1 = loaded['sol_dq'][:, 1]
    sol_dq_2 = loaded['sol_dq'][:, 2]
    sol_dq_3 = loaded['sol_dq'][:, 3]
    sol_dq_4 = loaded['sol_dq'][:, 4]
    sol_dq_5 = loaded['sol_dq'][:, 5]

    sol_acc_0 = loaded['sol_acc'][:, 0]
    sol_acc_1 = loaded['sol_acc'][:, 1]
    sol_acc_2 = loaded['sol_acc'][:, 2]
    sol_acc_3 = loaded['sol_acc'][:, 3]
    sol_acc_4 = loaded['sol_acc'][:, 4]
    sol_acc_5 = loaded['sol_acc'][:, 5]

    sol_tau_0 = loaded['sol_tau'][:, 0]
    sol_tau_1 = loaded['sol_tau'][:, 1]
    sol_tau_2 = loaded['sol_tau'][:, 2]
    sol_tau_3 = loaded['sol_tau'][:, 3]
    sol_tau_4 = loaded['sol_tau'][:, 4]
    sol_tau_5 = loaded['sol_tau'][:, 5]

    # fig2, (ax21, ax22, ax23, ax24, ax25, ax26) = plt.subplots(nrows=6, ncols=1)
    #
    # ax21 = plt.subplot(611)
    # plt.plot(time, dq_0)
    # ax21.set_ylabel('dq 0')
    #
    # ax22 = plt.subplot(612)
    # plt.plot(time, dq_1)
    # ax22.set_ylabel('dq 1')
    #
    # ax23 = plt.subplot(613)
    # plt.plot(time, dq_2)
    # ax23.set_ylabel('dq 2')
    #
    # ax24 = plt.subplot(614)
    # plt.plot(time, dq_3)
    # ax24.set_ylabel('dq 3')
    #
    # ax25 = plt.subplot(615)
    # plt.plot(time, dq_4)
    # ax25.set_ylabel('dq 4')
    #
    # ax26 = plt.subplot(616)
    # plt.plot(time, dq_5)
    # ax26.set_ylabel('dq 5')
    # plt.xlabel('Time [s]')
    #
    # plt.grid(True)

    fig20 = plt.figure()
    ax20 = fig20.gca()
    ax20.plot(time, q_0, label='q 0')
    ax20.plot(time, q_1, label='q 1')
    ax20.plot(time, q_2, label='q 2')
    ax20.plot(time, q_3, label='q 3')
    ax20.plot(time, q_4, label='q 4')
    ax20.plot(time, q_5, label='q 5')
    ax20.legend(frameon=False, loc='upper left', prop=fontP)
    plt.ylabel('Joint positions [Radian]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    fig20.savefig("joint_positions.pdf", bbox_inches='tight')



    fig21 = plt.figure()
    ax = fig21.gca()
    ax.plot(time, dq_0, label='dq 0')
    ax.plot(time, dq_1, label='dq 1')
    ax.plot(time, dq_2, label='dq 2')
    ax.plot(time, dq_3, label='dq 3')
    ax.plot(time, dq_4, label='dq 4')
    ax.plot(time, dq_5, label='dq 5')
    ax.legend(frameon=False, loc='upper left', prop=fontP)
    plt.ylabel('Joint velocities [Radion/s]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    fig21.savefig("joint_velocities.pdf", bbox_inches='tight')


    fig22 = plt.figure()
    ax22 = fig22.gca()
    ax22.plot(time, acc_0, label='acc 0')
    ax22.plot(time, acc_1, label='acc 1')
    ax22.plot(time, acc_2, label='acc 2')
    ax22.plot(time, acc_3, label='acc 3')
    ax22.plot(time, acc_4, label='acc 4')
    ax22.plot(time, acc_5, label='acc 5')
    ax22.legend(frameon=False, loc='upper left', prop=fontP)
    plt.ylabel('Joint accelerations [Radion/s^2]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    fig22.savefig("joint_accelerations.pdf", bbox_inches='tight')

    fig23 = plt.figure()
    ax23 = fig23.gca()
    ax23.plot(time[0:len(tau_0)], tau_0, label='tau 0')
    ax23.plot(time[0:len(tau_0)], tau_1, label='tau 1')
    ax23.plot(time[0:len(tau_0)], tau_2, label='tau 2')
    ax23.plot(time[0:len(tau_0)], tau_3, label='tau 3')
    ax23.plot(time[0:len(tau_0)], tau_4, label='tau 4')
    ax23.plot(time[0:len(tau_0)], tau_5, label='tau 5')
    ax23.legend(frameon=False, loc='upper left', prop=fontP)
    plt.ylabel('Joint torque [Nm]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    fig23.savefig("joint_torques.pdf", bbox_inches='tight')

    fig24 = plt.figure()
    ax24 = fig24.gca()
    ax24.plot(time[0:len(tau_0)], sol_acc_0, label='sol 0')
    ax24.plot(time[0:len(tau_0)], sol_acc_1, label='sol 1')
    ax24.plot(time[0:len(tau_0)], sol_acc_2, label='sol 2')
    ax24.plot(time[0:len(tau_0)], sol_acc_3, label='sol 3')
    ax24.plot(time[0:len(tau_0)], sol_acc_4, label='sol 4')
    ax24.plot(time[0:len(tau_0)], sol_acc_5, label='sol 5')
    ax24.legend(frameon=False, loc='upper left', prop=fontP)
    plt.ylabel('QP: Joint acceleration [Radion/s^2]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    fig24.savefig("solution_acc.pdf", bbox_inches='tight')

    fig25 = plt.figure()
    ax25 = fig25.gca()
    ax25.plot(time[0:len(tau_0)], sol_dq_0, label='sol 0')
    ax25.plot(time[0:len(tau_0)], sol_dq_1, label='sol 1')
    ax25.plot(time[0:len(tau_0)], sol_dq_2, label='sol 2')
    ax25.plot(time[0:len(tau_0)], sol_dq_3, label='sol 3')
    ax25.plot(time[0:len(tau_0)], sol_dq_4, label='sol 4')
    ax25.plot(time[0:len(tau_0)], sol_dq_5, label='sol 5')
    ax25.legend(frameon=False, loc='upper left', prop=fontP)

    plt.ylabel('QP: Joint Velocity [Radion/s]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    fig25.savefig("solution_dq.pdf", bbox_inches='tight')

    fig26 = plt.figure()
    ax26 = fig26.gca()
    ax26.plot(time[0:len(tau_0)], sol_q_0, label='sol 0')
    ax26.plot(time[0:len(tau_0)], sol_q_1, label='sol 1')
    ax26.plot(time[0:len(tau_0)], sol_q_2, label='sol 2')
    ax26.plot(time[0:len(tau_0)], sol_q_3, label='sol 3')
    ax26.plot(time[0:len(tau_0)], sol_q_4, label='sol 4')
    ax26.plot(time[0:len(tau_0)], sol_q_5, label='sol 5')
    ax26.legend(frameon=False, loc='upper left', prop=fontP)
    plt.ylabel('QP: Joint Position [Radion]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    fig26.savefig("solution_q.pdf", bbox_inches='tight')


    fig27 = plt.figure()
    ax27 = fig27.gca()
    ax27.plot(time[0:len(tau_0)], sol_tau_0, label='sol 0')
    ax27.plot(time[0:len(tau_0)], sol_tau_1, label='sol 1')
    ax27.plot(time[0:len(tau_0)], sol_tau_2, label='sol 2')
    ax27.plot(time[0:len(tau_0)], sol_tau_3, label='sol 3')
    ax27.plot(time[0:len(tau_0)], sol_tau_4, label='sol 4')
    ax27.plot(time[0:len(tau_0)], sol_tau_5, label='sol 5')
    ax27.legend(frameon=False, loc='upper left', prop=fontP)
    plt.ylabel('QP: Joint Torque [Nm]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    fig27.savefig("solution_torque.pdf", bbox_inches='tight')


    # fig3, (ax31, ax32, ax33, ax34, ax35, ax36) = plt.subplots(nrows=6, ncols=1)
    #
    # ax31 = plt.subplot(611)
    # plt.plot(time, rc_0)
    # ax31.set_ylabel('robot c 0')
    #
    # ax32 = plt.subplot(612)
    # plt.plot(time, rc_1)
    # ax32.set_ylabel('robot c 1')
    #
    # ax33 = plt.subplot(613)
    # plt.plot(time, rc_2)
    # ax33.set_ylabel('robot c 2')
    #
    # ax34 = plt.subplot(614)
    # plt.plot(time, rc_3)
    # ax34.set_ylabel('robot c 3')
    #
    # ax35 = plt.subplot(615)
    # plt.plot(time, rc_4)
    # ax35.set_ylabel('robot c 4')
    #
    # ax36 = plt.subplot(616)
    # plt.plot(time, rc_5)
    # ax36.set_ylabel('robot c 5')

    # fig31 = plt.figure()
    # ax = fig31.gca()
    # ax.plot(time, rc_0, label='rc 0')
    # ax.plot(time, rc_1, label='rc 1')
    # ax.plot(time, rc_2, label='rc 2')
    # ax.plot(time, rc_3, label='rc 3')
    # ax.plot(time, rc_4, label='rc 4')
    # ax.plot(time, rc_5, label='rc 5')
    # ax.legend()
    # plt.ylabel('C(q, dq) + G')
    # plt.xlabel('Time [s]')
    # plt.grid(True)


    plt.show()

