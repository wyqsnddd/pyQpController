import sys
import numpy as np


import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from matplotlib.ticker import FormatStrFormatter
from matplotlib.font_manager import FontProperties

mpl.rcParams['ps.useafm'] = True
mpl.rcParams['pdf.use14corefonts'] = True
mpl.rcParams['text.usetex'] = True


# from matplotlib import colors as mcolors
# colors = dict(mcolors.BASE_COLORS, **mcolors.CSS4_COLORS)


if __name__ =="__main__":

    fileName =  sys.argv[1]
    loaded = np.load(fileName)

    #loaded = np.load("../log/data/jointVelocityJump-data_Jan_31_2019_16-09-14.npz")
    #loaded = np.load("../log/data/data_Jan_31_2019_18-50-43.npz")

    time = loaded['time']

    # impact_time_1 = [0.24, 0.245] # Generic QP impact case
    # impact_time_1 = [0.815, 0.825]
    # impact_time_1 = [0.80, 0.815]
    impact_time_1 = [0.795, 0.805]
    impact_time_2 = [0.485, 0.495]
    impact_time_3 = [0.72, 0.73]
    impact_time_4 = [0.99, 1.0]

    font = {'family' : 'normal',
            'weight' : 'bold',
            'size'   : 15}

    fontP = FontProperties()
    fontP.set_size('small')

    dq = loaded['dq']
    tau = loaded['tau']
    ee_v = loaded['ee_v']
    ee_f = loaded['ee_f']
    f_QP = loaded['f_QP']
    length = len(time)
    f_desired = np.ones((length ,1))*57
    
    
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

    y_bins = 4
    x_bins = 14

    
    fig4, (ax41, ax42, ax43, ax44, ax45, ax46) = plt.subplots(nrows=6, ncols=1, figsize=(12,6))
    ax41 = plt.subplot(611)
    ax41.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    plt.plot(time, ddq_upper_bound_velocity_0, 'r--', label='Upper bound under impacts')
    plt.plot(time, ddq_lower_bound_velocity_0, 'g--', label='Lower bound under impacts')
    plt.plot(time, ddq_0, 'b', label='$\\ddot{q}_0$')
    plt.plot(time, real_ddq_upper_bound_velocity_0, 'r', label='Upper bound')
    plt.plot(time, real_ddq_lower_bound_velocity_0, 'g', label='Lower bound')

    ax41.locator_params(nbins=6, axis='y')
    ax41.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax41.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.title("Converted joint velocity constraints   [$radian/s^2$]")
    plt.yticks(fontsize=10)
    ax41.locator_params(nbins=y_bins, axis='y')
    ax41.legend(loc='upper left', prop={'size':5}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1, handlelength=4 )
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)

    # ax41.autoscale(enable=True, axis='y')
    ax41.set_ylim([-1500, 1500])

    
    ax42 = plt.subplot(612)
    plt.plot(time, ddq_upper_bound_velocity_1, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_1, 'g--')
    plt.plot(time, ddq_1, 'b', label='$\\ddot{q}_1$')
    plt.plot(time, real_ddq_upper_bound_velocity_1, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_1, 'g')
    ax42.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax42.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax42.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax42.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)

    ax42.autoscale(enable=True, axis='y')
    # ax42.set_ylim([-1200, 1200])
    ax42.legend(loc='upper left', prop={'size':5}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )

    ax43 = plt.subplot(613)
    plt.plot(time, ddq_upper_bound_velocity_2, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_2, 'g--')
    plt.plot(time, ddq_2, 'b', label='$\\ddot{q}_2$')
    plt.plot(time, real_ddq_upper_bound_velocity_2, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_2, 'g')
    ax43.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax43.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax43.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax43.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)
    
    ax43.autoscale(enable=True, axis='y')
    # ax43.set_ylim([-1600, 1500])
    ax43.legend(loc='upper left', prop={'size':5}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    ax44 = plt.subplot(614)
    plt.plot(time, ddq_upper_bound_velocity_3, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_3, 'g--')
    plt.plot(time, ddq_3, 'b', label='$\\ddot{q}_3$')
    plt.plot(time, real_ddq_upper_bound_velocity_3, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_3, 'g')
    ax44.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax44.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax44.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax44.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)

    ax44.autoscale(enable=True, axis='y')
    # ax44.set_ylim([-120000, 120000])
    ax44.set_ylim([-1600, 1600])
    ax44.legend(loc='upper left', prop={'size':5}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    ax45 = plt.subplot(615)
    plt.plot(time, ddq_upper_bound_velocity_4, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_4, 'g--')
    plt.plot(time, ddq_4, 'b', label='$\\ddot{q}_4$')
    plt.plot(time, real_ddq_upper_bound_velocity_4, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_4, 'g')
    ax45.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax45.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax45.get_xticklabels(), visible=False)
    plt.grid(True)
    plt.yticks(fontsize=10)
    ax45.locator_params(nbins=y_bins, axis='y')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)

    
    ax45.autoscale(enable=True, axis='y')
    ax45.set_ylim([-1800, 3000])
    ax45.legend(loc='upper left', prop={'size':5}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    
    ax46 = plt.subplot(616)
    plt.plot(time, ddq_upper_bound_velocity_5, 'r--')
    plt.plot(time, ddq_lower_bound_velocity_5, 'g--')
    plt.plot(time, ddq_5, 'b', label='$\\ddot{q}_5$')
    plt.plot(time, real_ddq_upper_bound_velocity_5, 'r')
    plt.plot(time, real_ddq_lower_bound_velocity_5, 'g')
    plt.xlabel('Time [s]')
    ax46.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax46.autoscale(enable=True, axis='x', tight=True)
    plt.grid(True)
    plt.yticks(fontsize=10)
    # ax46.set_ylabel('$\\ddot{q}_5$', **font)
    ax46.locator_params(nbins=y_bins, axis='y')
    ax46.locator_params(nbins=x_bins, axis='x')
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)

    ax46.autoscale(enable=True, axis='y')
    ax42.set_ylim([-2200, 2200])
    ax46.legend(loc='upper left', prop={'size':5}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    fig4.savefig("ddq_velocity_impact_bounds.pdf", bbox_inches='tight')


    fig7 = plt.figure(figsize=(12,4))
    lower_bound = -25*np.ones(len(ddq_5))
    upper_bound = 25*np.ones(len(ddq_5))
    y_bins = 8
    x_bins = 14

    ax71 = fig7.gca()
    ax71.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # plt.plot(time, predict_tauUpper[:,0], 'r--', label='Maximum torque under impacts')
    # plt.plot(time, predict_tauLower[:,0], 'g--', label='Minimum torque under impacts')
    # plt.plot(time, upper_bound - predict_tauUpper[:,0], 'r', label='Upper bound: Torque')
    # plt.plot(time, lower_bound + predict_tauLower[:,0], 'g', label='Lower bound: Torque')
    plt.plot(time, upper_bound, 'red', linestyle='-', label='Upper bound: $\overline{\\tau} $', linewidth=2.0)
    plt.plot(time, lower_bound, 'darkslategrey', linestyle='-', label='Lower bound: $ \underline{\\tau} $', linewidth=2.0)

    ax71.locator_params(nbins=y_bins, axis='y')
    ax71.locator_params(nbins=x_bins, axis='x')
    plt.setp(ax71.get_xticklabels(), visible=False)
    plt.grid(True)
    ax71.legend(frameon=False, loc='lower left', prop=fontP)
    plt.title("Joint torque constraints [$Nm$]")
    
    plt.yticks(fontsize=10)
    ax71.locator_params(nbins=y_bins, axis='y')


    plt.plot(time, tau[:,0], 'mediumblue', label='Torque: $\\tau_0 $')
    plt.plot(time, tau[:,1], 'indigo', label='Torque: $\\tau_1 $')
    plt.plot(time, tau[:,2], 'magenta', label='Torque: $\\tau_2 $')
    plt.plot(time, tau[:,3], 'crimson', label='Torque: $\\tau_3 $')
    plt.plot(time, tau[:,4], 'peru', label='Torque: $\\tau_4 $')
    plt.plot(time, tau[:,5], 'darkorange', label='Torque: $\\tau_5 $')

    
    ax71.legend(loc='upper left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 , handlelength=4)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    ax71.set_ylim([-28, 28])
    ax71.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    plt.xlabel('Time [$s$]')
    plt.setp(ax71.get_xticklabels(), visible=True)
    ax71.autoscale(enable=True, axis='x', tight=True)

    
    plt.grid(True)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    fig7.savefig("All_torque_impact_bounds.pdf", bbox_inches='tight')

    

    
    

    fig7 = plt.figure(figsize=(6,2))
    # fig8, (ax80, ax81) = plt.subplots(nrows=2, ncols=1)
    # ax80 =  plt.subplot(211)
    ax80 = fig7.gca()
    # set up subplot grid
    gridspec.GridSpec(4,1)
    # plt.subplot2grid((4,1), (0,0), colspan=1, rowspan=1)
    plt.plot(time, ee_v[:,0], 'b', label='$v_{n}$')
    plt.title("Contact velocity  [$m/s$] ", fontsize=10)
    ax80.autoscale(enable=True, axis='y', tight=True)

    ax80.autoscale(enable=True, axis='x', tight=True)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)

    plt.grid(True)
    ax80.locator_params(nbins=6, axis='y')
    plt.yticks(fontsize=10)
    plt.xticks(fontsize=10)
    # ax80.legend(prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=2, handlelength=4 )
    plt.xlabel('Time [s]', fontsize=10)
    ax71.set_xlim([0, 1.2])
    # plt.text(1.2, 0.16, r'$v_{n} = 0.163 m/s$',
    #          {'color': 'k', 'fontsize': 10, 'ha': 'center', 'va': 'center',
    #           'bbox': dict(boxstyle="round", fc="w", ec="k", pad=0.2)})
    plt.text(1.05, 0.155, r'$v_{n} = 0.155 m/s$',
             {'color': 'k', 'fontsize': 10, 'ha': 'center', 'va': 'center',
              'bbox': dict(boxstyle="round", fc="w", ec="k", pad=0.2)})
    plt.annotate("", xy=(0.9, 0.155), xycoords='data',
                 xytext=(0.8, 0.155), textcoords='data',
                 arrowprops=dict(arrowstyle="<-", connectionstyle="arc3"))

    fig7.savefig("contact_velocity.pdf", bbox_inches='tight')

    # fig8.set_figheight(3)
    # fig8.set_figwidth(10)

    
    fig8 = plt.figure(figsize=(6,5))
    
    lower =-200
    upper = 200
    lower_bound = lower*np.ones(len(ddq_5))
    upper_bound = upper*np.ones(len(ddq_5))

    # lower_bound = -25*np.ones(len(ddq_5))
    # upper_bound = 25*np.ones(len(ddq_5))


    y_bins = 8
    x_bins = 12
    ax81 = fig8.gca()
    # plt.subplot2grid((4,1), (1,0), colspan=1, rowspan=3)

    ax81.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax81.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    plt.plot(time, upper_bound, 'red', linestyle='-.', label='Upper  bound: $\delta \overline{\\tau} $ ', linewidth=2.0)
    plt.plot(time, lower_bound, 'darkslategrey', linestyle='-.', label=r'Lower  bound: $\delta   \underline{\tau}$ ', linewidth=2.0)
    ax81.set_xlim([0, 1.2])
    
    # ax81.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax81.get_xticklabels(), visible=False)
    plt.grid(True)
    ax81.set_ylim([1.05*lower, 50])
    # ax81.set_ylim([1.15*lower, 1.15*upper])
    # ax81.set_ylim([4.5*lower, 4.5*upper])
    plt.title("Impulse joint torque constraints [$Nm$]", fontsize=10)
    # ax81.set_ylabel('Joint Torque  [$Nm$]', **font)
    plt.yticks(fontsize=10)
    plt.xticks(fontsize=10)
    ax81.locator_params(nbins=y_bins, axis='y')
    ax81.locator_params(nbins=x_bins, axis='x')

    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)

    plt.axhline(y=-200, xmin=0.02, xmax=0.66, linewidth=8, color='green', alpha=0.2)

    
    plt.plot(time, predict_impulseTau[:,0], color='mediumblue', linestyle='--', label='Predicted: $\delta \\tau_0$')
    plt.plot(time, impulseTau[:,0], 'mediumblue', label='Torque: $\\tau_0$')

    # ax82 = plt.subplot(612)
    # plt.yticks(fontsize=10)
    
    plt.plot(time, predict_impulseTau[:,1], color='indigo', linestyle='--', label='Predicted: $\delta \\tau_1$')
    plt.plot(time, impulseTau[:,1], 'indigo', label='Torque: $\\tau_0$')
    
    # plt.plot(time, upper_bound, 'r')
    # plt.plot(time, lower_bound, 'g')
    # ax82.set_ylabel('$\delta \\tau_1$', **font)
    # ax82.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax82.autoscale(enable=True, axis='x', tight=True)
    # ax82.locator_params(nbins=y_bins, axis='y')
    # plt.setp(ax82.get_xticklabels(), visible=False)
    # plt.grid(True)
    # ax82.set_ylim([lower - 20, upper + 20])
    # plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    # ax83 = plt.subplot(613)
    # plt.yticks(fontsize=10)
    plt.plot(time, predict_impulseTau[:,2], color='magenta', linestyle='--', label='Predicted:  $\delta \\tau_2$')
    plt.plot(time, impulseTau[:,2], 'magenta', label='Torque: $\\tau_2$')
    # plt.plot(time, upper_bound, 'r')
    # plt.plot(time, lower_bound, 'g')
    # ax83.set_ylabel('$\delta \\tau_2$', **font)
    # ax83.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax83.autoscale(enable=True, axis='x', tight=True)
    # ax83.locator_params(nbins=y_bins, axis='y')
    # plt.setp(ax83.get_xticklabels(), visible=False)
    # plt.grid(True)
    # ax83.set_ylim([lower - 20, upper + 20])
    # plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    # ax84 = plt.subplot(614)
    # plt.yticks(fontsize=10)
    plt.plot(time, predict_impulseTau[:,3], color='crimson', linestyle='--', label='Predicted:  $\delta \\tau_3$')
    plt.plot(time, impulseTau[:,3], 'crimson', label='Torque:  $\\tau_3$')
    # plt.plot(time, upper_bound, 'r')
    # plt.plot(time, lower_bound, 'g')
    # ax84.set_ylabel('$\delta \\tau_3$', **font)
    # ax84.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax84.autoscale(enable=True, axis='x', tight=True)
    # ax84.locator_params(nbins=y_bins, axis='y')
    # plt.setp(ax84.get_xticklabels(), visible=False)
    # plt.grid(True)
    # ax84.set_ylim([lower - 20, upper + 20])
    # plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    # ax85 = plt.subplot(615)
    # plt.yticks(fontsize=10)
    plt.plot(time, predict_impulseTau[:,4], color='peru', linestyle='--', label='Predicted:  $\delta \\tau_4$')
    plt.plot(time, impulseTau[:,4], 'peru', label='Torque:  $\\tau_4$')
    # plt.plot(time, upper_bound, 'r')
    # plt.plot(time, lower_bound, 'g')
    # ax85.set_ylabel('$\delta \\tau_4$', **font)
    # ax85.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax85.autoscale(enable=True, axis='x', tight=True)
    # ax85.locator_params(nbins=y_bins, axis='y')
    # plt.setp(ax85.get_xticklabels(), visible=False)
    # plt.grid(True)
    # ax85.set_ylim([lower - 20, upper + 20])
    # plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)

    # ax86 = plt.subplot(616)
    # plt.yticks(fontsize=10)
    plt.plot(time, predict_impulseTau[:,5], color='darkorange', linestyle='--', label='Predicted:  $\delta \\tau_5$')
    plt.plot(time, impulseTau[:,5], 'darkorange', label='Torque:  $\\tau_5$')
    # plt.plot(time, upper_bound, 'r')
    # plt.plot(time, lower_bound, 'g')
    # ax86.set_ylabel('$\delta \\tau_5$', **font)
    plt.xlabel('Time [$s$]')

    ax81.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    # ax81.autoscale(enable=True, axis='x', tight=True)
    # ax86.locator_params(nbins=y_bins, axis='y')
    # plt.grid(True)
    # ax86.set_ylim([lower - 20, upper + 20])
    # plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    # ax81.legend(loc='lower left', prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=2, handlelength=4 )
    ax81.legend(loc='lower right', prop={'size':4}, fancybox=True, framealpha=0.3, shadow=False, borderpad=2, handlelength=4 )
    ax81.autoscale(enable=True, axis='x', tight=True)
    plt.setp(ax81.get_xticklabels(), visible=True)


    # # Arrow
    # plt.annotate("", xy=(0.24, -200), xycoords='data',
    #              xytext=(0.6, -800), textcoords='data',
    #              arrowprops=dict(arrowstyle="->", connectionstyle="arc3"))
    # plt.annotate("", xy=(0.485, -200), xycoords='data',
    #              xytext=(0.6, -800), textcoords='data',
    #              arrowprops=dict(arrowstyle="->", connectionstyle="arc3"))
    # plt.annotate("", xy=(0.72, -200), xycoords='data',
    #              xytext=(0.6, -800), textcoords='data',
    #              arrowprops=dict(arrowstyle="->", connectionstyle="arc3"))
    # plt.annotate("", xy=(0.99, -200), xycoords='data',
    #              xytext=(0.6, -800), textcoords='data',
    #              arrowprops=dict(arrowstyle="->", connectionstyle="arc3"))
    # # Text
    # plt.text(0.6, -800, r'Violation of $\delta   \underline{\tau} $',
    #          {'color': 'k', 'fontsize': 10, 'ha': 'center', 'va': 'center',
    #           'bbox': dict(boxstyle="round", fc="w", ec="k", pad=0.2)})

    plt.annotate("", xy=(0.6, -200), xycoords='data',
                 xytext=(0.6, -120), textcoords='data',
                 arrowprops=dict(arrowstyle="->", connectionstyle="arc3"))

    plt.text(0.5, -110, r' Predicted $\delta \tau_3$ slides along $\delta   \underline{\tau}$',
             {'color': 'k', 'fontsize': 10, 'ha': 'center', 'va': 'center',
              'bbox': dict(boxstyle="round", fc="w", ec="k", pad=0.2)})

    
    fig8.savefig("all_impulse_torque_bounds.pdf", bbox_inches='tight')



    fig9, (ax80, ax81) = plt.subplots(nrows=2, ncols=1, figsize=(6,4))
    ax80 = plt.subplot(211)
    # set up subplot grid
    # plt.subplot2grid((4,1), (0,0), colspan=1, rowspan=1)
    plt.plot(time, ee_v[:,0], 'b', label='$v_{n}$')
    plt.title("Contact velocity  [$m/s$] ", fontsize=10)
    ax80.autoscale(enable=True, axis='y', tight=True)

    ax80.autoscale(enable=True, axis='x', tight=True)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)

    plt.grid(True)
    ax80.locator_params(nbins=6, axis='y')
    plt.yticks(fontsize=10)
    plt.xticks(fontsize=10)
    # ax80.legend(prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=2, handlelength=4 )
    plt.xlabel('Time [s]', fontsize=10)

    ax81 = plt.subplot(212)
    plt.plot(time, -ee_f[:,0], 'g', label='$f$')
    plt.plot(time, f_QP[:,0], 'r', label='$f_{QP}$')
    plt.plot(time, f_desired, 'b', linestyle='-.', label='$f^*$')
    
    plt.title("Contact force $f_{x}$  [$N$] ", fontsize=10)
    ax81.autoscale(enable=True, axis='y', tight=True)

    ax81.autoscale(enable=True, axis='x', tight=True)
    plt.axvspan(impact_time_1[0], impact_time_1[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_2[0], impact_time_2[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_3[0], impact_time_3[1], color='red', alpha=0.1)
    # plt.axvspan(impact_time_4[0], impact_time_4[1], color='red', alpha=0.1)

    plt.grid(True)
    ax81.locator_params(nbins=6, axis='y')
    plt.yticks(fontsize=10)
    plt.xticks(fontsize=10)
    # ax80.legend(prop={'size':6}, fancybox=True, framealpha=0.3, shadow=False, borderpad=2, handlelength=4 )
    plt.xlabel('Time [s]', fontsize=10)
    plt.subplots_adjust(hspace=0.5)
    ax81.legend(loc='upper left', prop={'size':5}, fancybox=True, framealpha=0.3, shadow=False, borderpad=1 )
    # # Arrow
    # plt.annotate("", xy=(1.0, 600), xycoords='data',
    #              xytext=(1.0, 0.0), textcoords='data',
    #              arrowprops=dict(arrowstyle="<->", connectionstyle="arc3"))
    # # Indication line: 
    # plt.annotate("", xy=(0.8, 600), xycoords='data',
    #              xytext=(1.2, 600.0), textcoords='data',
    #              arrowprops=dict(arrowstyle="-", connectionstyle="arc3"))
    # plt.text(1.0, 300, r'599.04 N',
    #      {'color': 'k', 'fontsize': 10, 'ha': 'center', 'va': 'center',
    #       'bbox': dict(boxstyle="round", fc="w", ec="k", pad=0.2)})
    ax81.set_ylim([-10, 510])

    fig9.savefig("contact_velocity_and_force.pdf", bbox_inches='tight')

    
    plt.show()
