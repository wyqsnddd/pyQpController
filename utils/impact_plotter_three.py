import sys
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from matplotlib.font_manager import FontProperties


# We want to plot the allowable joint velocity jumps for each joint.

if __name__ =="__main__":

    fileName =  sys.argv[1]
    loaded = np.load(fileName)

    # loaded = np.load("../log/data/data_Nov_03_2018_15-40-26.npz")

    time = loaded['time']


    fontP = FontProperties()
    fontP.set_size('small')


    predict_delta_dq_upper_0 = loaded['predict_jointVelocityJump_upper'][:,0]
    predict_delta_dq_upper_1 = loaded['predict_jointVelocityJump_upper'][:,1]
    predict_delta_dq_upper_2 = loaded['predict_jointVelocityJump_upper'][:,2]
    predict_delta_dq_upper_3 = loaded['predict_jointVelocityJump_upper'][:,3]
    predict_delta_dq_upper_4 = loaded['predict_jointVelocityJump_upper'][:,4]
    predict_delta_dq_upper_5 = loaded['predict_jointVelocityJump_upper'][:,5]

    predict_delta_dq_lower_0 = loaded['predict_jointVelocityJump_lower'][:,0]
    predict_delta_dq_lower_1 = loaded['predict_jointVelocityJump_lower'][:,1]
    predict_delta_dq_lower_2 = loaded['predict_jointVelocityJump_lower'][:,2]
    predict_delta_dq_lower_3 = loaded['predict_jointVelocityJump_lower'][:,3]
    predict_delta_dq_lower_4 = loaded['predict_jointVelocityJump_lower'][:,4]
    predict_delta_dq_lower_5 = loaded['predict_jointVelocityJump_lower'][:,5]





    fig2, (ax21, ax22, ax23, ax24, ax25, ax26) = plt.subplots(nrows=6, ncols=1)

    ax21 = plt.subplot(611)
    ax21.set_ylabel('dq 0')
    ax21.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    plt.plot(time, predict_delta_dq_upper_0, label='Upper bound')
    plt.plot(time, predict_delta_dq_lower_0, label='Lower bound')
    ax21.locator_params(nbins=5, axis='y')
    ax21.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax21.get_xticklabels(), visible=False)
    plt.grid(True)
    ax21.legend(frameon=False, loc='upper left', prop=fontP)
    plt.title("Range of the estimated joint velocities jumps [Radian/Second]")

    ax22 = plt.subplot(612)
    plt.plot(time, predict_delta_dq_upper_1)
    plt.plot(time, predict_delta_dq_lower_1)
    ax22.set_ylabel('dq 1')
    ax22.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax22.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax22.get_xticklabels(), visible=False)
    plt.grid(True)
    ax22.locator_params(nbins=5, axis='y')

    ax23 = plt.subplot(613)
    plt.plot(time, predict_delta_dq_upper_2)
    plt.plot(time, predict_delta_dq_lower_2)
    ax23.set_ylabel('dq 2')
    ax23.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax23.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax23.get_xticklabels(), visible=False)
    plt.grid(True)
    ax23.locator_params(nbins=5, axis='y')

    ax24 = plt.subplot(614)
    plt.plot(time, predict_delta_dq_upper_3)
    plt.plot(time, predict_delta_dq_lower_3)
    ax24.set_ylabel('dq 3')
    plt.setp(ax24.get_xticklabels(), visible=False)
    ax24.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax24.autoscale(enable=True, axis='x', tight=True)

    plt.grid(True)
    ax24.legend()
    ax24.locator_params(nbins=5, axis='y')

    ax25 = plt.subplot(615)
    plt.plot(time, predict_delta_dq_upper_4)
    plt.plot(time, predict_delta_dq_lower_4)
    ax25.set_ylabel('dq 4')
    plt.setp(ax25.get_xticklabels(), visible=False)
    ax25.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax25.autoscale(enable=True, axis='x', tight=True)
    plt.grid(True)
    ax25.legend()
    ax25.locator_params(nbins=5, axis='y')

    ax26 = plt.subplot(616)
    plt.plot(time, predict_delta_dq_upper_5)
    plt.plot(time, predict_delta_dq_lower_5)
    ax26.set_ylabel('dq 5')
    plt.xlabel('Time [s]')
    plt.grid(True)
    ax26.legend()
    ax26.locator_params(nbins=5, axis='y')
    ax26.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax26.autoscale(enable=True, axis='x', tight=True)
    fig2.savefig("Delta_dq_bounds.pdf", bbox_inches='tight')
    
    plt.show()
