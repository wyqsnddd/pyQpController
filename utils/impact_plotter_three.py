import sys
import numpy as np

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from matplotlib.font_manager import FontProperties

if __name__ =="__main__":

    fileName =  sys.argv[1]
    loaded = np.load(fileName)

    #loaded = np.load("../log/data/jointVelocityJump-data_Nov_26_2018_16-22-04.npz")

    time = loaded['time']

    impact_time_1 = [0.62, 0.635]

    font = {'family' : 'normal',
            'weight' : 'bold',
            'size'   : 15}

    fontP = FontProperties()
    fontP.set_size('small')

    dq = loaded['dq']
    tau = loaded['tau']
    predict_tauUpper = loaded['predict_tauUpper']
    predict_tauLower = loaded['predict_tauLower']
    predict_impulseTau = loaded['predict_impulseTau']
    impulseTau = loaded['impulseTau']
    
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


    ddq_upper_bound_position_0 = loaded['ddqUpperBoundPosition'][:,0]
    ddq_upper_bound_position_1 = loaded['ddqUpperBoundPosition'][:,1]
    ddq_upper_bound_position_2 = loaded['ddqUpperBoundPosition'][:,2]
    ddq_upper_bound_position_3 = loaded['ddqUpperBoundPosition'][:,3]
    ddq_upper_bound_position_4 = loaded['ddqUpperBoundPosition'][:,4]
    ddq_upper_bound_position_5 = loaded['ddqUpperBoundPosition'][:,5]

    ddq_lower_bound_position_0 = loaded['ddqLowerBoundPosition'][:,0]
    ddq_lower_bound_position_1 = loaded['ddqLowerBoundPosition'][:,1]
    ddq_lower_bound_position_2 = loaded['ddqLowerBoundPosition'][:,2]
    ddq_lower_bound_position_3 = loaded['ddqLowerBoundPosition'][:,3]
    ddq_lower_bound_position_4 = loaded['ddqLowerBoundPosition'][:,4]
    ddq_lower_bound_position_5 = loaded['ddqLowerBoundPosition'][:,5]

    ddq_upper_bound_velocity_0 = loaded['ddqUpperBoundVelocity'][:,0]
    ddq_upper_bound_velocity_1 = loaded['ddqUpperBoundVelocity'][:,1]
    ddq_upper_bound_velocity_2 = loaded['ddqUpperBoundVelocity'][:,2]
    ddq_upper_bound_velocity_3 = loaded['ddqUpperBoundVelocity'][:,3]
    ddq_upper_bound_velocity_4 = loaded['ddqUpperBoundVelocity'][:,4]
    ddq_upper_bound_velocity_5 = loaded['ddqUpperBoundVelocity'][:,5]

    ddq_lower_bound_velocity_0 = loaded['ddqLowerBoundVelocity'][:,0]
    ddq_lower_bound_velocity_1 = loaded['ddqLowerBoundVelocity'][:,1]
    ddq_lower_bound_velocity_2 = loaded['ddqLowerBoundVelocity'][:,2]
    ddq_lower_bound_velocity_3 = loaded['ddqLowerBoundVelocity'][:,3]
    ddq_lower_bound_velocity_4 = loaded['ddqLowerBoundVelocity'][:,4]
    ddq_lower_bound_velocity_5 = loaded['ddqLowerBoundVelocity'][:,5]

    real_ddq_upper_bound_position_0 = loaded['real_ddqUpperBoundPosition'][:,0]
    real_ddq_upper_bound_position_1 = loaded['real_ddqUpperBoundPosition'][:,1]
    real_ddq_upper_bound_position_2 = loaded['real_ddqUpperBoundPosition'][:,2]
    real_ddq_upper_bound_position_3 = loaded['real_ddqUpperBoundPosition'][:,3]
    real_ddq_upper_bound_position_4 = loaded['real_ddqUpperBoundPosition'][:,4]
    real_ddq_upper_bound_position_5 = loaded['real_ddqUpperBoundPosition'][:,5]

    real_ddq_lower_bound_position_0 = loaded['real_ddqLowerBoundPosition'][:,0]
    real_ddq_lower_bound_position_1 = loaded['real_ddqLowerBoundPosition'][:,1]
    real_ddq_lower_bound_position_2 = loaded['real_ddqLowerBoundPosition'][:,2]
    real_ddq_lower_bound_position_3 = loaded['real_ddqLowerBoundPosition'][:,3]
    real_ddq_lower_bound_position_4 = loaded['real_ddqLowerBoundPosition'][:,4]
    real_ddq_lower_bound_position_5 = loaded['real_ddqLowerBoundPosition'][:,5]

    real_ddq_upper_bound_velocity_0 = loaded['real_ddqUpperBoundVelocity'][:,0]
    real_ddq_upper_bound_velocity_1 = loaded['real_ddqUpperBoundVelocity'][:,1]
    real_ddq_upper_bound_velocity_2 = loaded['real_ddqUpperBoundVelocity'][:,2]
    real_ddq_upper_bound_velocity_3 = loaded['real_ddqUpperBoundVelocity'][:,3]
    real_ddq_upper_bound_velocity_4 = loaded['real_ddqUpperBoundVelocity'][:,4]
    real_ddq_upper_bound_velocity_5 = loaded['real_ddqUpperBoundVelocity'][:,5]

    real_ddq_lower_bound_velocity_0 = loaded['real_ddqLowerBoundVelocity'][:,0]
    real_ddq_lower_bound_velocity_1 = loaded['real_ddqLowerBoundVelocity'][:,1]
    real_ddq_lower_bound_velocity_2 = loaded['real_ddqLowerBoundVelocity'][:,2]
    real_ddq_lower_bound_velocity_3 = loaded['real_ddqLowerBoundVelocity'][:,3]
    real_ddq_lower_bound_velocity_4 = loaded['real_ddqLowerBoundVelocity'][:,4]
    real_ddq_lower_bound_velocity_5 = loaded['real_ddqLowerBoundVelocity'][:,5]

    real_ddq_upper_bound_tau_0 = loaded['real_ddqUpperBoundTau'][:,0]
    real_ddq_upper_bound_tau_1 = loaded['real_ddqUpperBoundTau'][:,1]
    real_ddq_upper_bound_tau_2 = loaded['real_ddqUpperBoundTau'][:,2]
    real_ddq_upper_bound_tau_3 = loaded['real_ddqUpperBoundTau'][:,3]
    real_ddq_upper_bound_tau_4 = loaded['real_ddqUpperBoundTau'][:,4]
    real_ddq_upper_bound_tau_5 = loaded['real_ddqUpperBoundTau'][:,5]

    real_ddq_lower_bound_tau_0 = loaded['real_ddqLowerBoundTau'][:,0]
    real_ddq_lower_bound_tau_1 = loaded['real_ddqLowerBoundTau'][:,1]
    real_ddq_lower_bound_tau_2 = loaded['real_ddqLowerBoundTau'][:,2]
    real_ddq_lower_bound_tau_3 = loaded['real_ddqLowerBoundTau'][:,3]
    real_ddq_lower_bound_tau_4 = loaded['real_ddqLowerBoundTau'][:,4]
    real_ddq_lower_bound_tau_5 = loaded['real_ddqLowerBoundTau'][:,5]

    predict_ddq_upper_bound_tau_0 = loaded['predict_ddqUpperBoundTau'][:,0]
    predict_ddq_upper_bound_tau_1 = loaded['predict_ddqUpperBoundTau'][:,1]
    predict_ddq_upper_bound_tau_2 = loaded['predict_ddqUpperBoundTau'][:,2]
    predict_ddq_upper_bound_tau_3 = loaded['predict_ddqUpperBoundTau'][:,3]
    predict_ddq_upper_bound_tau_4 = loaded['predict_ddqUpperBoundTau'][:,4]
    predict_ddq_upper_bound_tau_5 = loaded['predict_ddqUpperBoundTau'][:,5]

    ddq_0 = loaded['ddq'][:, 0]
    ddq_1 = loaded['ddq'][:, 1]
    ddq_2 = loaded['ddq'][:, 2]
    ddq_3 = loaded['ddq'][:, 3]
    ddq_4 = loaded['ddq'][:, 4]
    ddq_5 = loaded['ddq'][:, 5]

    predict_ddq_lower_bound_tau_0 = loaded['predict_ddqLowerBoundTau'][:,0]
    predict_ddq_lower_bound_tau_1 = loaded['predict_ddqLowerBoundTau'][:,1]
    predict_ddq_lower_bound_tau_2 = loaded['predict_ddqLowerBoundTau'][:,2]
    predict_ddq_lower_bound_tau_3 = loaded['predict_ddqLowerBoundTau'][:,3]
    predict_ddq_lower_bound_tau_4 = loaded['predict_ddqLowerBoundTau'][:,4]
    predict_ddq_lower_bound_tau_5 = loaded['predict_ddqLowerBoundTau'][:,5]


    
    fig2, (ax21, ax22, ax23, ax24, ax25, ax26) = plt.subplots(nrows=6, ncols=1)
    y_bins = 5
    ax21 = plt.subplot(611)
    ax21.set_ylabel('dq 0')
    ax21.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    plt.plot(time, predict_delta_dq_upper_0, label='$\\delta \dot{q}$')
    plt.plot(time, predict_delta_dq_lower_0, label='Lower bound')
    plt.plot(time, dq[:,0], label='$\dot{q}$')
    ax21.locator_params(nbins=y_bins, axis='y')
    ax21.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax21.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.title("Comparison between  $\dot{q}$ and $\\delta \dot{q}$   [$radian/s$]")

    plt.yticks(fontsize=10)
    ax21.locator_params(nbins=y_bins, axis='y')
    ax21.set_ylabel('$\\dot{q}_0$', **font)
    # ax21.locator_params(nbins=y_bins, axis='y')
    ax21.legend(loc='upper left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    
    ax22 = plt.subplot(612)
    plt.plot(time, predict_delta_dq_upper_1)
    plt.plot(time, predict_delta_dq_lower_1)
    plt.plot(time, dq[:,1])
    ax22.set_ylabel('dq 1')
    ax22.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax22.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax22.get_xticklabels(), visible=False)
    plt.grid(True)
    ax22.locator_params(nbins=y_bins, axis='y')
    
    plt.yticks(fontsize=10)
    ax22.locator_params(nbins=y_bins, axis='y')
    ax22.set_ylabel('$\\dot{q}_1$', **font)
    # ax21.locator_params(nbins=y_bins, axis='y')
    ax22.legend(loc='upper left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    
    ax23 = plt.subplot(613)
    plt.plot(time, predict_delta_dq_upper_2)
    plt.plot(time, predict_delta_dq_lower_2)
    plt.plot(time, dq[:, 2])

    ax23.set_ylabel('dq 2')
    ax23.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax23.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax23.get_xticklabels(), visible=False)
    plt.grid(True)
    ax23.locator_params(nbins=y_bins, axis='y')
    plt.yticks(fontsize=10)
    ax23.locator_params(nbins=y_bins, axis='y')
    ax23.set_ylabel('$\\dot{q}_2$', **font)
    # ax21.locator_params(nbins=y_bins, axis='y')
    ax23.legend(loc='upper left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    ax24 = plt.subplot(614)
    plt.plot(time, predict_delta_dq_upper_3)
    plt.plot(time, predict_delta_dq_lower_3)
    plt.plot(time, dq[:, 3])

    ax24.set_ylabel('dq 3')
    plt.setp(ax24.get_xticklabels(), visible=False)
    ax24.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax24.autoscale(enable=True, axis='x', tight=True)

    plt.grid(True)
    ax24.legend()
    ax24.locator_params(nbins=y_bins, axis='y')

    plt.yticks(fontsize=10)
    ax24.locator_params(nbins=y_bins, axis='y')
    ax24.set_ylabel('$\\dot{q}_3$', **font)
    # ax21.locator_params(nbins=y_bins, axis='y')
    ax24.legend(loc='upper left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    
    ax25 = plt.subplot(615)
    plt.plot(time, predict_delta_dq_upper_4)
    plt.plot(time, predict_delta_dq_lower_4)
    plt.plot(time, dq[:, 4])

    ax25.set_ylabel('dq 4')
    plt.setp(ax25.get_xticklabels(), visible=False)
    ax25.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax25.autoscale(enable=True, axis='x', tight=True)
    plt.grid(True)
    ax25.legend()
    ax25.locator_params(nbins=y_bins, axis='y')

    plt.yticks(fontsize=10)
    ax25.locator_params(nbins=y_bins, axis='y')
    ax25.set_ylabel('$\\dot{q}_4$', **font)
    ax21.locator_params(nbins=y_bins, axis='y')
    ax25.legend(loc='upper left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    
    ax26 = plt.subplot(616)
    plt.plot(time, predict_delta_dq_upper_5)
    plt.plot(time, predict_delta_dq_lower_5)
    plt.plot(time, dq[:, 5])

    ax26.set_ylabel('dq 5')
    plt.xlabel('Time [s]')
    plt.grid(True)
    ax26.legend()
    ax26.locator_params(nbins=y_bins, axis='y')
    ax26.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
    ax26.autoscale(enable=True, axis='x', tight=True)
    plt.yticks(fontsize=10)
    ax26.locator_params(nbins=y_bins, axis='y')
    ax26.set_ylabel('$\\dot{q}_5$', **font)
    # ax21.locator_params(nbins=y_bins, axis='y')
    ax26.legend(loc='upper left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    
    fig2.savefig("Delta_dq_bounds.pdf", bbox_inches='tight')
    








    fig3, (ax31, ax32, ax33, ax34, ax35, ax36) = plt.subplots(nrows=6, ncols=1)
    y_bins = 5
    
    ax31 = plt.subplot(611)
    ax31.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    plt.plot(time, ddq_upper_bound_position_0, 'r--', label='Upper bound under impact: Position')
    plt.plot(time, ddq_lower_bound_position_0, 'g--', label='Lower bound under impact: Position')
    plt.plot(time, ddq_0, 'b', label='QP solution ')
    plt.plot(time, real_ddq_upper_bound_position_0, 'r', label='Upper bound: Position')
    plt.plot(time, real_ddq_lower_bound_position_0, 'g', label='Lower bound: Position')

    # ax31.locator_params(nbins=y_bins, axis='y')
    ax31.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax31.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.title("Converted joint position constraints  [$radian/s^2$]")
    plt.yticks(fontsize=10)
    ax31.set_ylabel('$\\ddot{q}_0$', **font)
    ax31.locator_params(nbins=y_bins, axis='y')
    ax31.legend(loc='upper left', prop={'size':5}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    # ax31.autoscale(enable=True, axis='y')
    ax31.set_ylim([-150000, 150000])

    ax32 = plt.subplot(612)
    plt.plot(time, ddq_upper_bound_position_1, 'r--')
    plt.plot(time, ddq_lower_bound_position_1, 'g--')
    plt.plot(time, ddq_1, 'b')
    plt.plot(time, real_ddq_upper_bound_position_1, 'r')
    plt.plot(time, real_ddq_lower_bound_position_1, 'g')
    ax32.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax32.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax32.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax32.set_ylabel('$\\ddot{q}_1$', **font)
    ax32.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    # ax32.autoscale(enable=True, axis='y')
    ax32.set_ylim([-120000, 120000])

    ax33 = plt.subplot(613)
    plt.plot(time, ddq_upper_bound_position_2, 'r--')
    plt.plot(time, ddq_lower_bound_position_2, 'g--')
    plt.plot(time, ddq_2, 'b')
    plt.plot(time, real_ddq_upper_bound_position_2, 'r')
    plt.plot(time, real_ddq_lower_bound_position_2, 'g')
    ax33.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax33.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax33.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax33.set_ylabel('$\\ddot{q}_2$', **font)
    ax33.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax33.autoscale(enable=True, axis='y')
    ax33.set_ylim([-150000, 110000])

    ax34 = plt.subplot(614)
    plt.plot(time, ddq_upper_bound_position_3, 'r--')
    plt.plot(time, ddq_lower_bound_position_3, 'g--')
    plt.plot(time, ddq_3, 'b')
    plt.plot(time, real_ddq_upper_bound_position_3, 'c')
    plt.plot(time, real_ddq_lower_bound_position_3, 'g')
    ax34.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax34.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax34.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax34.set_ylabel('$\\ddot{q}_3$', **font)
    ax34.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax34.autoscale(enable=True, axis='y')
    ax34.set_ylim([-150000, 150000])

    ax35 = plt.subplot(615)
    plt.plot(time, ddq_upper_bound_position_4, 'r--')
    plt.plot(time, ddq_lower_bound_position_4, 'g--')
    plt.plot(time, ddq_4, 'b')
    plt.plot(time, real_ddq_upper_bound_position_4, 'r')
    plt.plot(time, real_ddq_lower_bound_position_4, 'g')
    ax35.set_ylabel('ddq 4')
    ax35.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax35.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax35.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax35.set_ylabel('$\\ddot{q}_4$', **font)
    ax35.locator_params(nbins=3, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax35.autoscale(enable=True, axis='y')
    ax35.set_ylim([-100000, 120000])

    ax36 = plt.subplot(616)
    plt.plot(time, ddq_upper_bound_position_5, 'r--')
    plt.plot(time, ddq_lower_bound_position_5, 'g--')
    plt.plot(time, ddq_5, 'b')
    plt.plot(time, real_ddq_upper_bound_position_5, 'r')
    plt.plot(time, real_ddq_lower_bound_position_5, 'g')
    ax36.set_ylabel('ddq 5')
    plt.xlabel('Time [$s$]')
    ax36.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax36.autoscale(enable=True, axis='x', tight=True)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax36.set_ylabel('$\\ddot{q}_5$', **font)
    ax36.locator_params(nbins=5, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax36.autoscale(enable=True, axis='y')
    ax36.set_ylim([-300000, 300000])

    fig3.savefig("ddq_position_impact_bounds.pdf", bbox_inches='tight')
    
    fig4, (ax41, ax42, ax43, ax44, ax45, ax46) = plt.subplots(nrows=6, ncols=1)
    ax41 = plt.subplot(611)
    ax41.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    plt.plot(time, ddq_upper_bound_velocity_0, 'r--', label='Upper bound under impacts: Velocity')
    plt.plot(time, ddq_lower_bound_velocity_0, 'g--', label='Lower bound under impacts: Velocity')
    plt.plot(time, ddq_0, 'b')
    plt.plot(time, real_ddq_upper_bound_velocity_0, 'r', label='Upper bound: Velocity')
    plt.plot(time, real_ddq_lower_bound_velocity_0, 'g', label='Lower bound: Velocity')

    ax41.locator_params(nbins=6, axis='y')
    ax41.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax41.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.title("Converted joint velocity constraints   [$radian/s^2$]")
    plt.yticks(fontsize=10)
    ax41.set_ylabel('$\\ddot{q}_0$', **font)
    ax41.locator_params(nbins=y_bins, axis='y')
    ax41.legend(loc='upper left', prop={'size':5}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    # ax41.autoscale(enable=True, axis='y')
    ax41.set_ylim([-1500, 1500])

    
    ax42 = plt.subplot(612)
    plt.plot(time, ddq_upper_bound_velocity_1, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_1, 'g--')
    plt.plot(time, ddq_1, 'b')
    plt.plot(time, real_ddq_upper_bound_velocity_1, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_1, 'g')
    ax42.set_ylabel('ddq 1')
    ax42.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax42.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax42.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax42.set_ylabel('$\\ddot{q}_1$', **font)
    ax42.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax42.autoscale(enable=True, axis='y')
    ax42.set_ylim([-1200, 1200])

    ax43 = plt.subplot(613)
    plt.plot(time, ddq_upper_bound_velocity_2, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_2, 'g--')
    plt.plot(time, ddq_2, 'b')
    plt.plot(time, real_ddq_upper_bound_velocity_2, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_2, 'g')
    ax43.set_ylabel('ddq 2')
    ax43.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax43.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax43.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax43.set_ylabel('$\\ddot{q}_2$', **font)
    ax43.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax43.autoscale(enable=True, axis='y')
    ax43.set_ylim([-1600, 1500])
    
    ax44 = plt.subplot(614)
    plt.plot(time, ddq_upper_bound_velocity_3, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_3, 'g--')
    plt.plot(time, ddq_3, 'b')
    plt.plot(time, real_ddq_upper_bound_velocity_3, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_3, 'g')
    ax44.set_ylabel('ddq 3')
    ax44.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax44.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax44.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax44.set_ylabel('$\\ddot{q}_3$', **font)
    ax44.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax44.autoscale(enable=True, axis='y')
    # ax42.set_ylim([-120000, 120000])
    
    ax45 = plt.subplot(615)
    plt.plot(time, ddq_upper_bound_velocity_4, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_4, 'g--')
    plt.plot(time, ddq_4, 'b')
    plt.plot(time, real_ddq_upper_bound_velocity_4, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_4, 'g')
    ax45.set_ylabel('ddq 4')
    ax45.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax45.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax45.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax45.set_ylabel('$\\ddot{q}_4$', **font)
    ax45.locator_params(nbins=3, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax45.autoscale(enable=True, axis='y')
    ax45.set_ylim([-1800, 3000])

    ax46 = plt.subplot(616)
    plt.plot(time, ddq_upper_bound_velocity_5, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_5, 'g--')
    plt.plot(time, ddq_5, 'b')
    plt.plot(time, real_ddq_upper_bound_velocity_5, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_5, 'g')
    ax46.set_ylabel('ddq 5')
    plt.xlabel('Time [s]')
    ax46.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax46.autoscale(enable=True, axis='x', tight=True)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax46.set_ylabel('$\\ddot{q}_5$', **font)
    ax46.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax46.autoscale(enable=True, axis='y')
    # ax42.set_ylim([-120000, 120000])

    fig4.savefig("ddq_velocity_impact_bounds.pdf", bbox_inches='tight')




    # fig5, (ax51, ax52, ax53, ax54, ax55, ax56) = plt.subplots(nrows=6, ncols=1)
    # ax51 = plt.subplot(611)
    # ax51.set_ylabel('ddq 0')
    # ax51.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # plt.plot(time, predict_ddq_upper_bound_tau_0, 'r--', label='Upper bound under impacts: Torque')
    # plt.plot(time, predict_ddq_lower_bound_tau_0, 'g--', label='Lower bound under impacts: Torque')
    # plt.plot(time, ddq_0, 'b', label='QP solution')
    # plt.plot(time, real_ddq_upper_bound_tau_0, 'r', label='Upper bound: Torque')
    # plt.plot(time, real_ddq_lower_bound_tau_0, 'g', label='Lower bound: Torque')

    # ax51.locator_params(nbins=6, axis='y')
    # ax51.autoscale(enable=True, axis='x', tight=True)
    # plt.setp(ax51.get_xticklabels(), visible=False)
    # plt.grid(True)
    # ax51.legend(frameon=False, loc='upper left', prop=fontP)
    # plt.title("Bounds on joint accelerations under impacts [Radian/Second^2]")

    # ax52 = plt.subplot(612)
    # plt.plot(time, predict_ddq_upper_bound_tau_1, 'r--')
    # plt.plot(time, predict_ddq_lower_bound_tau_1, 'g--')
    # plt.plot(time, ddq_1, 'b')
    # plt.plot(time, real_ddq_upper_bound_tau_1, 'r')
    # plt.plot(time, real_ddq_lower_bound_tau_1, 'g')
    # ax52.set_ylabel('ddq 1')
    # ax52.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax52.autoscale(enable=True, axis='x', tight=True)
    # plt.setp(ax52.get_xticklabels(), visible=False)
    # plt.grid(True)

    # ax53 = plt.subplot(613)
    # plt.plot(time, predict_ddq_upper_bound_tau_2, 'r--')
    # plt.plot(time, predict_ddq_lower_bound_tau_2, 'g--')
    # plt.plot(time, ddq_2, 'b')
    # plt.plot(time, real_ddq_upper_bound_tau_2, 'r')
    # plt.plot(time, real_ddq_lower_bound_tau_2, 'g')
    # ax53.set_ylabel('ddq 2')
    # ax53.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax53.autoscale(enable=True, axis='x', tight=True)
    # plt.setp(ax53.get_xticklabels(), visible=False)
    # plt.grid(True)

    # ax54 = plt.subplot(614)
    # plt.plot(time, predict_ddq_upper_bound_tau_3, 'r--')
    # plt.plot(time, predict_ddq_lower_bound_tau_3, 'g--')
    # plt.plot(time, ddq_3, 'b')
    # plt.plot(time, real_ddq_upper_bound_tau_3, 'r')
    # plt.plot(time, real_ddq_lower_bound_tau_3, 'g')
    # ax54.set_ylabel('ddq 3')
    # ax54.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax54.autoscale(enable=True, axis='x', tight=True)
    # plt.setp(ax54.get_xticklabels(), visible=False)
    # plt.grid(True)

    # ax55 = plt.subplot(615)
    # plt.plot(time, predict_ddq_upper_bound_tau_4, 'r--')
    # plt.plot(time, predict_ddq_lower_bound_tau_4, 'g--')
    # plt.plot(time, ddq_4, 'b')
    # plt.plot(time, real_ddq_upper_bound_tau_4, 'r')
    # plt.plot(time, real_ddq_lower_bound_tau_4, 'g')
    # ax55.set_ylabel('ddq 4')
    # ax55.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax55.autoscale(enable=True, axis='x', tight=True)
    # plt.setp(ax55.get_xticklabels(), visible=False)
    # plt.grid(True)

    # ax56 = plt.subplot(616)
    # plt.plot(time, predict_ddq_upper_bound_tau_5, 'r--')
    # plt.plot(time, predict_ddq_lower_bound_tau_5, 'g--')
    # plt.plot(time, ddq_5, 'b')
    # plt.plot(time, real_ddq_upper_bound_tau_5, 'r')
    # plt.plot(time, real_ddq_lower_bound_tau_5, 'g')
    # ax56.set_ylabel('ddq 5')
    # plt.xlabel('Time [s]')
    # ax56.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax56.autoscale(enable=True, axis='x', tight=True)
    # plt.grid(True)
    # fig5.savefig("ddq_tau_impact_bounds.pdf", bbox_inches='tight')

    acc_upper_0 = []
    acc_lower_0 = []
    predict_acc_upper_0 = []
    predict_acc_lower_0 = []
    for ii in range(0, len(predict_ddq_lower_bound_tau_0)):
        acc_upper_0.append(min(real_ddq_upper_bound_position_0[ii], real_ddq_upper_bound_velocity_0[ii])) 
        acc_lower_0.append(max(real_ddq_lower_bound_position_0[ii], real_ddq_lower_bound_velocity_0[ii]))
        predict_acc_upper_0.append(min(ddq_upper_bound_position_0[ii], ddq_upper_bound_velocity_0[ii]))
        predict_acc_lower_0.append(max(ddq_lower_bound_position_0[ii], ddq_lower_bound_velocity_0[ii]))

    acc_upper_1 = []
    acc_lower_1 = []
    predict_acc_upper_1 = []
    predict_acc_lower_1 = []
    for ii in range(0, len(predict_ddq_lower_bound_tau_1)):
        acc_upper_1.append(min(real_ddq_upper_bound_position_1[ii], real_ddq_upper_bound_velocity_1[ii] ) )
        acc_lower_1.append(max(real_ddq_lower_bound_position_1[ii], real_ddq_lower_bound_velocity_1[ii] ) )
        predict_acc_upper_1.append(min(ddq_upper_bound_position_1[ii], ddq_upper_bound_velocity_1[ii]))
        predict_acc_lower_1.append(max(ddq_lower_bound_position_1[ii], ddq_lower_bound_velocity_1[ii]))

    acc_upper_2 = []
    acc_lower_2 = []
    predict_acc_upper_2 = []
    predict_acc_lower_2 = []
    for ii in range(0, len(predict_ddq_lower_bound_tau_2)):
        acc_upper_2.append(min(real_ddq_upper_bound_position_2[ii], real_ddq_upper_bound_velocity_2[ii] ) )
        acc_lower_2.append(max(real_ddq_lower_bound_position_2[ii], real_ddq_lower_bound_velocity_2[ii] ) )
        predict_acc_upper_2.append(min(ddq_upper_bound_position_2[ii], ddq_upper_bound_velocity_2[ii]))
        predict_acc_lower_2.append(max(ddq_lower_bound_position_2[ii], ddq_lower_bound_velocity_2[ii]))

    acc_upper_3 = []
    acc_lower_3 = []
    predict_acc_upper_3 = []
    predict_acc_lower_3 = []
    for ii in range(0, len(predict_ddq_lower_bound_tau_3)):
        acc_upper_3.append(min(real_ddq_upper_bound_position_3[ii], real_ddq_upper_bound_velocity_3[ii] ) )
        acc_lower_3.append(max(real_ddq_lower_bound_position_3[ii], real_ddq_lower_bound_velocity_3[ii] ) )
        predict_acc_upper_3.append(min(ddq_upper_bound_position_3[ii], ddq_upper_bound_velocity_3[ii]))
        predict_acc_lower_3.append(max(ddq_lower_bound_position_3[ii], ddq_lower_bound_velocity_3[ii]))

    acc_upper_4 = []
    acc_lower_4 = []
    predict_acc_upper_4 = []
    predict_acc_lower_4 = []
    for ii in range(0, len(predict_ddq_lower_bound_tau_4)):
        acc_upper_4.append(min(real_ddq_upper_bound_position_4[ii], real_ddq_upper_bound_velocity_4[ii] ) )
        acc_lower_4.append(max(real_ddq_lower_bound_position_4[ii], real_ddq_lower_bound_velocity_4[ii] ) )
        predict_acc_upper_4.append(min(ddq_upper_bound_position_4[ii], ddq_upper_bound_velocity_4[ii]))
        predict_acc_lower_4.append(max(ddq_lower_bound_position_4[ii], ddq_lower_bound_velocity_4[ii]))

    acc_upper_5 = []
    acc_lower_5 = []
    predict_acc_upper_5 = []
    predict_acc_lower_5 = []
    for ii in range(0, len(predict_ddq_lower_bound_tau_5)):
        acc_upper_5.append(min(real_ddq_upper_bound_position_5[ii], real_ddq_upper_bound_velocity_5[ii] ) )
        acc_lower_5.append(max(real_ddq_lower_bound_position_5[ii], real_ddq_lower_bound_velocity_5[ii] ) )
        predict_acc_upper_5.append(min(ddq_upper_bound_position_5[ii], ddq_upper_bound_velocity_5[ii]))
        predict_acc_lower_5.append(max(ddq_lower_bound_position_5[ii], ddq_lower_bound_velocity_5[ii]))


    fig6, (ax61, ax62, ax63, ax64, ax65, ax66) = plt.subplots(nrows=6, ncols=1)
    ax61 = plt.subplot(611)
    ax61.set_ylabel('ddq 0')
    ax61.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    plt.plot(time, acc_upper_0, 'r', label='Upper bound ')
    plt.plot(time, predict_acc_upper_0, 'r--', label='Predicted upper bound under impact ')
    plt.plot(time, ddq_0, 'b', label='QP solution ')
    plt.plot(time, acc_lower_0, 'g', label='Lower bound ')
    plt.plot(time, predict_acc_lower_0, 'g--', label='Predicted lower bound under impact ')

    ax61.locator_params(nbins=6, axis='y')
    ax61.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax61.get_xticklabels(), visible=False)
    plt.grid(True)
    ax61.legend(frameon=False, loc='upper left', prop=fontP)
    plt.title("Bounds on joint accelerations under impacts [Radian/Second^2]")

    ax62 = plt.subplot(612)
    plt.plot(time, acc_upper_1, 'r')
    plt.plot(time, predict_acc_upper_1, 'r--')
    plt.plot(time, ddq_1, 'b')
    plt.plot(time, acc_lower_1, 'g')
    plt.plot(time, predict_acc_lower_1, 'g--')
    ax62.set_ylabel('ddq 1')
    ax62.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax62.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax62.get_xticklabels(), visible=False)
    plt.grid(True)

    ax63 = plt.subplot(613)
    plt.plot(time, acc_upper_2, 'r')
    plt.plot(time, predict_acc_upper_2, 'r--')
    plt.plot(time, ddq_2, 'b')
    plt.plot(time, acc_lower_2, 'g')
    plt.plot(time, predict_acc_lower_2, 'g--')
    ax63.set_ylabel('ddq 2')
    ax63.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax63.autoscale(enable=True, axis='x', tight=True) # 
    plt.setp(ax63.get_xticklabels(), visible=False)
    plt.grid(True)

    ax64 = plt.subplot(614)
    plt.plot(time, acc_upper_3, 'r')
    plt.plot(time, predict_acc_upper_3, 'r--')
    plt.plot(time, ddq_3, 'b')
    plt.plot(time, acc_lower_3, 'g')
    plt.plot(time, predict_acc_lower_3, 'g--')
    ax64.set_ylabel('ddq 3')
    ax64.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax64.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax64.get_xticklabels(), visible=False)
    plt.grid(True)

    ax65 = plt.subplot(615)
    plt.plot(time, acc_upper_4, 'r')
    plt.plot(time, predict_acc_upper_4, 'r--')
    plt.plot(time, ddq_4, 'b')
    plt.plot(time, acc_lower_4, 'g')
    plt.plot(time, predict_acc_lower_4, 'g--')
    ax65.set_ylabel('ddq 4')
    ax65.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax65.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax65.get_xticklabels(), visible=False)
    plt.grid(True)

    ax66 = plt.subplot(616)
    plt.plot(time, acc_upper_5, 'r')
    plt.plot(time, predict_acc_upper_5, 'r--')
    plt.plot(time, ddq_5, 'b')
    plt.plot(time, acc_lower_5, 'g')
    plt.plot(time, predict_acc_lower_5, 'g--')
    ax66.set_ylabel('ddq 5')
    plt.xlabel('Time [s]')
    ax66.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax66.autoscale(enable=True, axis='x', tight=True)
    plt.grid(True)
    fig6.savefig("ddq_bounds_comparison.pdf", bbox_inches='tight')

    fig7, (ax71, ax72, ax73, ax74, ax75, ax76) = plt.subplots(nrows=6, ncols=1)
    lower_bound = -25*np.ones(len(ddq_5))
    upper_bound = 25*np.ones(len(ddq_5))
    
    ax71 = plt.subplot(611)
    ax71.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # plt.plot(time, predict_tauUpper[:,0], 'r--', label='Maximum torque under impacts')
    # plt.plot(time, predict_tauLower[:,0], 'g--', label='Minimum torque under impacts')
    plt.plot(time, tau[:,0], 'b', label='QP solution torque')
    # plt.plot(time, upper_bound - predict_tauUpper[:,0], 'r', label='Upper bound: Torque')
    # plt.plot(time, lower_bound + predict_tauLower[:,0], 'g', label='Lower bound: Torque')
    plt.plot(time, upper_bound, 'r', label='Upper bound')
    plt.plot(time, lower_bound, 'g', label='Lower bound')

    ax71.locator_params(nbins=6, axis='y')
    ax71.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax71.get_xticklabels(), visible=False)
    plt.grid(True)
    ax71.legend(frameon=False, loc='upper left', prop=fontP)
    plt.title("Joint torque constraints [$Nm$]")
    
    plt.yticks(fontsize=10)
    ax71.set_ylabel('$\\tau_0$', **font)
    ax71.locator_params(nbins=y_bins, axis='y')
    ax71.legend(loc='upper left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax71.set_ylim([-28, 28])


    
    ax72 = plt.subplot(612)
    # plt.plot(time, predict_tauUpper[:,1], 'r--')
    # plt.plot(time, predict_tauLower[:,1], 'g--')
    plt.plot(time, tau[:,1], 'b')
    # plt.plot(time, upper_bound - predict_tauUpper[:,1], 'r')
    # plt.plot(time, lower_bound + predict_tauLower[:,1], 'g')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax72.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax72.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax72.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax72.locator_params(nbins=y_bins, axis='y')
    ax72.set_ylabel('$\\tau_1$', **font)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax72.set_ylim([-28, 28])
    
    ax73 = plt.subplot(613)
    plt.plot(time, predict_tauUpper[:,2], 'r--')
    plt.plot(time, predict_tauLower[:,2], 'g--')
    plt.plot(time, tau[:,2], 'b')
    # plt.plot(time, upper_bound - predict_tauUpper[:,2], 'r')
    # plt.plot(time, lower_bound + predict_tauLower[:,2], 'g')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax73.set_ylabel('Torque 2')
    ax73.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax73.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax73.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax73.locator_params(nbins=y_bins, axis='y')
    ax73.set_ylabel('$\\tau_2$', **font)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax73.set_ylim([-28, 28])

    ax74 = plt.subplot(614)
    plt.plot(time, predict_tauUpper[:,3], 'r--')
    plt.plot(time, predict_tauLower[:,3], 'g--')
    plt.plot(time, tau[:,3], 'b')
    # plt.plot(time, upper_bound - predict_tauUpper[:,3], 'r')
    # plt.plot(time, lower_bound + predict_tauLower[:,3], 'g')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax74.set_ylabel('Torque 3')
    ax74.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax74.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax74.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax74.locator_params(nbins=y_bins, axis='y')
    ax74.set_ylabel('$\\tau_3$', **font)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax74.set_ylim([-28, 28])

    ax75 = plt.subplot(615)
    plt.plot(time, predict_tauUpper[:,4], 'r--')
    plt.plot(time, predict_tauLower[:,4], 'g--')
    plt.plot(time, tau[:,4], 'b')
    # plt.plot(time, upper_bound - predict_tauUpper[:,4], 'r')
    # plt.plot(time, lower_bound + predict_tauLower[:,4], 'g')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax75.set_ylabel('Torque 4')
    ax75.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax75.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax75.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax75.locator_params(nbins=y_bins, axis='y')
    ax75.set_ylabel('$\\tau_4$', **font)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax75.set_ylim([-28, 28])

    ax76 = plt.subplot(616)
    plt.plot(time, predict_tauUpper[:,5], 'r--')
    plt.plot(time, predict_tauLower[:,5], 'g--')
    plt.plot(time, tau[:,5], 'b')
    # plt.plot(time, upper_bound - predict_tauUpper[:,5], 'r')
    # plt.plot(time, lower_bound + predict_tauLower[:,5], 'g')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax76.set_ylabel('Torque 5')
    plt.xlabel('Time [s]')
    ax76.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax76.autoscale(enable=True, axis='x', tight=True)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax76.locator_params(nbins=y_bins, axis='y')
    ax76.set_ylabel('$\\tau_5$', **font)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax76.set_ylim([-28, 28])

    fig7.savefig("Torque_impact_bounds.pdf", bbox_inches='tight')

    fig8, (ax81, ax82, ax83, ax84, ax85, ax86) = plt.subplots(nrows=6, ncols=1)


    lower =-200
    upper = 200
    lower_bound = lower*np.ones(len(ddq_5))
    upper_bound = upper*np.ones(len(ddq_5))

    # lower_bound = -25*np.ones(len(ddq_5))
    # upper_bound = 25*np.ones(len(ddq_5))


    y_bins = 5
    ax81 = plt.subplot(611)

    ax81.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    plt.plot(time, predict_impulseTau[:,0], 'b--', label='Predicted impulse torque')
    plt.plot(time, impulseTau[:,0], 'b', label='Measured torque')
    plt.plot(time, upper_bound, 'r', label='Upper impulse torque bound')
    plt.plot(time, lower_bound, 'g', label='Lower impulse torque bound')
    ax81.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax81.get_xticklabels(), visible=False)
    plt.grid(True)
    ax81.set_ylim([lower - 20, upper + 20])
    plt.title("Impulse joint torque constraints [$Nm$]")
    
    plt.yticks(fontsize=10)
    ax81.set_ylabel('$\delta \\tau_0$', **font)
    ax81.locator_params(nbins=y_bins, axis='y')
    ax81.legend(loc='upper left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    ax82 = plt.subplot(612)
    plt.yticks(fontsize=10)
    plt.plot(time, predict_impulseTau[:,1], 'b--')
    plt.plot(time, impulseTau[:,1], 'b')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax82.set_ylabel('$\delta \\tau_1$', **font)
    ax82.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax82.autoscale(enable=True, axis='x', tight=True)
    ax82.locator_params(nbins=y_bins, axis='y')
    plt.setp(ax82.get_xticklabels(), visible=False)
    plt.grid(True)
    ax82.set_ylim([lower - 20, upper + 20])
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    ax83 = plt.subplot(613)
    plt.yticks(fontsize=10)
    plt.plot(time, predict_impulseTau[:,2], 'b--')
    plt.plot(time, impulseTau[:,2], 'b')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax83.set_ylabel('$\delta \\tau_2$', **font)
    ax83.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax83.autoscale(enable=True, axis='x', tight=True)
    ax83.locator_params(nbins=y_bins, axis='y')
    plt.setp(ax83.get_xticklabels(), visible=False)
    plt.grid(True)
    ax83.set_ylim([lower - 20, upper + 20])
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    ax84 = plt.subplot(614)
    plt.yticks(fontsize=10)
    plt.plot(time, predict_impulseTau[:,3], 'b--')
    plt.plot(time, impulseTau[:,3], 'b')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax84.set_ylabel('$\delta \\tau_3$', **font)
    ax84.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax84.autoscale(enable=True, axis='x', tight=True)
    ax84.locator_params(nbins=y_bins, axis='y')
    plt.setp(ax84.get_xticklabels(), visible=False)
    plt.grid(True)
    ax84.set_ylim([lower - 20, upper + 20])
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    ax85 = plt.subplot(615)
    plt.yticks(fontsize=10)
    plt.plot(time, predict_impulseTau[:,4], 'b--')
    plt.plot(time, impulseTau[:,4], 'b')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax85.set_ylabel('$\delta \\tau_4$', **font)
    ax85.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax85.autoscale(enable=True, axis='x', tight=True)
    ax85.locator_params(nbins=y_bins, axis='y')
    plt.setp(ax85.get_xticklabels(), visible=False)
    plt.grid(True)
    ax85.set_ylim([lower - 20, upper + 20])
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    ax86 = plt.subplot(616)
    plt.yticks(fontsize=10)
    plt.plot(time, predict_impulseTau[:,5], 'b--')
    plt.plot(time, impulseTau[:,5], 'b')
    plt.plot(time, upper_bound, 'r')
    plt.plot(time, lower_bound, 'g')
    ax86.set_ylabel('$\delta \\tau_5$', **font)
    plt.xlabel('Time [$s$]')
    ax86.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax86.autoscale(enable=True, axis='x', tight=True)
    ax86.locator_params(nbins=y_bins, axis='y')
    plt.grid(True)
    ax86.set_ylim([lower - 20, upper + 20])
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    fig8.savefig("impulse_torque_bounds.pdf", bbox_inches='tight')
    
    plt.show()
