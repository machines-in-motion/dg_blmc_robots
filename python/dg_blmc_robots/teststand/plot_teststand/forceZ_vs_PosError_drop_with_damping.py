import sys, os
from os.path import expanduser
from copy import deepcopy
import numpy as np
from numpy import math
import RAI
import pkg_resources
from scipy.signal import butter, lfilter, freqz, filtfilt

#
# Plot parameters
#
import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
font = {'family' : 'normal',
        # 'weight' : 'bold',
        'size'   : 22}
matplotlib.rc('font', **font)


#
# Helpful methods
#
def polyfit(x, y, degree):
    results = {}

    coeffs = np.polyfit(x, y, degree)

     # Polynomial Coefficients
    results['polynomial'] = coeffs.tolist()

    # r-squared
    p = np.poly1d(coeffs)
    # fit values, and mean
    yhat = p(x)                         # or [p(z) for z in x]
    ybar = np.sum(y)/len(y)          # or sum(y)/len(y)
    ssreg = np.sum((yhat-ybar)**2)   # or sum([ (yihat - ybar)**2 for yihat in yhat])
    sstot = np.sum((y - ybar)**2)    # or sum([ (yi - ybar)**2 for yi in y])
    results['determination'] = ssreg / sstot

    return results


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    # y = lfilter(b, a, data)
    y = filtfilt(b, a, data)
    return y


def get_data(rai_data, start_value, end_value):
  """
  Parse the data file
  """

  for field in rai_data.fields:
    print field

  Fx = np.array(rai_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[0]"))
  Fz = np.array(rai_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[2]"))
  Fz_filtered = np.array(rai_data.get_streams("data0/dg_stiffness_measurement_force_sensor-sout.dat[2]"))
  pos_error_x = np.array(rai_data.get_streams("data0/dg_pos_error_hopper-sout.dat[0]"))
  pos_error_z = np.array(rai_data.get_streams("data0/dg_pos_error_hopper-sout.dat[2]"))
  pos_error_height_sensor_filtered = np.array(rai_data.get_streams("data0/dg_stiffness_measurement_height_sensor-sout.dat[0]"))
  pos_error_height_sensor = np.array(rai_data.get_streams("data0/dg_hopper_teststand-height_sensors.dat[0]"))
  HFE_u = np.array(rai_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[0]"))
  KFE_u = np.array(rai_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[1]"))
  k_value_x = np.divide(Fx, pos_error_x)
  k_value_z = np.divide(Fz, pos_error_z)
  
  # experiment with free foot
  # pos_error_height_sensor_filtered = 0.22 + 0.017 - pos_error_height_sensor_filtered
  # pos_error_height_sensor = 0.22 + 0.017 - pos_error_height_sensor
  # Fz = 0.0 + Fz

  # experiment with attached foot
  pos_error_height_sensor_filtered = 0.28 - (pos_error_height_sensor_filtered + 0.007) 
  pos_error_height_sensor = 0.28 - (pos_error_height_sensor + 0.007)
  Fz = 0.0 + Fz

  Fx = Fx[start_value:end_value]
  Fz = Fz[start_value:end_value]
  Fz_filtered = Fz_filtered[start_value:end_value]
  pos_error_x = pos_error_x[start_value:end_value]
  pos_error_z = pos_error_z[start_value:end_value]
  k_value_x = k_value_x[start_value:end_value]
  k_value_z = k_value_z[start_value:end_value]
  HFE_u = HFE_u[start_value:end_value]
  KFE_u = KFE_u[start_value:end_value]
  pos_error_height_sensor = pos_error_height_sensor[start_value:end_value]
  pos_error_height_sensor_filtered = pos_error_height_sensor_filtered[start_value:end_value]

  return Fx, Fz, Fz_filtered, pos_error_x, pos_error_z, pos_error_height_sensor, pos_error_height_sensor_filtered, HFE_u, KFE_u, k_value_x, k_value_z

#
# Main plotting method
#
if __name__ == "__main__":
    """
    main function
    """
    #
    # Parse the arguments
    #
    
    folder = os.path.join(expanduser("~"),"fgrimminger_paper_log",
      "stiffness_data_compain_drop", "damping", "bare_ft_sensor")

    file_names = [
      os.path.join(folder, "2019-08-06_16-58-32", "2019-08-06_16-58-32.npz"),
      # os.path.join(folder, "2019-08-06_16-59-11", "2019-08-06_16-59-11.npz"),
      # os.path.join(folder, "2019-08-06_16-59-24", "2019-08-06_16-59-24.npz"),
     os.path.join(folder, "2019-08-06_17-00-05", "2019-08-06_17-00-05.npz"),
      # os.path.join(folder, "2019-08-06_17-00-12", "2019-08-06_17-00-12.npz"),
      # os.path.join(folder, "2019-08-06_17-00-20", "2019-08-06_17-00-20.npz"),
     os.path.join(folder, "2019-08-06_17-01-14", "2019-08-06_17-01-14.npz"),
      # os.path.join(folder, "2019-08-06_17-01-21", "2019-08-06_17-01-21.npz"),
      # os.path.join(folder, "2019-08-06_17-01-29", "2019-08-06_17-01-29.npz"),
    ]
    list_labels = [
      "k=100N/m trial 0",
      # "k=100N/m trial 1",
      # "k=100N/m trial 2",
      "k=200N/m trial 0",
      # "k=200N/m trial 1",
      # "k=200N/m trial 2",
      "k=300N/m trial 0",
      # "k=300N/m trial 1",
      # "k=300N/m trial 2",
    ]
    nb_files = len(file_names)

    print ("number of file to read: ", nb_files)
    print ("file names: ", file_names)
    print ("labels: ", list_labels)

    #
    # Create data lists
    #
    list_sensors_data = []
    list_data = []
    list_Fx = []
    list_Fz = []
    list_Fz_filtered = []
    list_pos_error_x = []
    list_pos_error_z = []
    list_pos_error_height_sensor = []
    list_pos_error_height_sensor_filtered = []
    list_HFE_u = []
    list_KFE_u = []
    list_k_value_x = []
    list_k_value_z = []
    list_Fz_cutoff = []
    list_pos_error_height_sensor_cutoff = []

    #
    # limit values
    # 
    start_value = 0
    end_value = -1

    #
    # Load the data
    #
    for i in range(nb_files):
        print("loading file : " + file_names[i])
        list_sensors_data.append(RAI.sensors.SensorsData(file_names[i]))
        list_data.append(list_sensors_data[-1].get_all_streams())

        (Fx, Fz, Fz_filtered, pos_error_x, pos_error_z, pos_error_height_sensor,
         pos_error_height_sensor_filtered, HFE_u, KFE_u,
         k_value_x, k_value_z) = get_data(list_sensors_data[i], start_value, end_value)

        # filter the force with a simple low pass filter
        # Filter requirements.
        order = 3
        fs = 1000.0     # sample rate, Hz
        cutoff = 50.0   # desired cutoff frequency of the filter, Hz
        Fz_cutoff = butter_lowpass_filter(Fz, cutoff, fs, order)
        pos_error_height_sensor_cutoff = butter_lowpass_filter(pos_error_height_sensor, cutoff, fs, order)

        # ignore the first 40 iteration (to make the filter converge)
        start_index = 0
        
        list_Fx.append(Fx[start_index:])
        list_Fz.append(Fz[start_index:])
        list_Fz_filtered.append(Fz_filtered[start_index:])
        list_pos_error_x.append(pos_error_x[start_index:])
        list_pos_error_z.append(pos_error_z[start_index:])
        list_pos_error_height_sensor.append(pos_error_height_sensor[start_index:])
        list_pos_error_height_sensor_filtered.append(pos_error_height_sensor_filtered[start_index:])
        list_HFE_u.append(HFE_u[start_index:])
        list_KFE_u.append(KFE_u[start_index:])
        list_k_value_x.append(k_value_x[start_index:])
        list_k_value_z.append(k_value_z[start_index:])
        list_Fz_cutoff.append(Fz_cutoff[start_index:])
        list_pos_error_height_sensor_cutoff.append(pos_error_height_sensor_cutoff[start_index:])

    #
    # cut the trajectories according to the impact
    #
    list_phase = [
      # k = 100
      [0, 1309, 1351, 1673, 1850],
      # k = 200
      [0, 345, 387, 610, 905],
      # k = 300
      [0, 765, 805, 997, 1330, 1517, 1662],
    ]
    for i in range(nb_files):
        list_phase[i].append(len(list_pos_error_height_sensor[i]))
    line_styles= [
      ['-', '--', '-', ':', '-'],
      ['-', '--', '-', ':', '-'],
      ['-', '--', '-', ':', '-', '--', '-']
    ]
    colors= [
      ['blue', 'dodgerblue', 'blue', 'turquoise', 'blue'],
      ['blue', 'dodgerblue', 'blue', 'turquoise', 'blue'],
      ['blue', 'dodgerblue', 'blue', 'turquoise', 'blue', 'lightseagreen', 'blue'],      
    ]
    labels= [
      ['', 'First Impact', 'Stable Regime', 'Second Impact', ''],
      ['', 'First Impact', 'Stable Regime', 'Second Impact', ''],
      ['', 'First Impact', 'Stable Regime', 'Second Impact', '', 'Third Impact', ''],
    ]

    #
    # Prepare plot
    #

    #
    # Drop test Fz(t)
    #
    # for i in range(nb_files):
    #     fig = plt.figure("Vertical Forces, k=" + str((i+1)*100) + "N/m")
    #     # plt.plot(list_pos_error_height_sensor[i], list_Fz[i], label = list_labels[i])
    #     plt.plot(list_Fz[i], label = list_labels[i])
    #     # plt.plot(list_Fz_cutoff[i], label = list_labels[i] + " cutoff")
    #     # plt.plot(list_Fz_filtered[i], label = list_labels[i] + " filtered")
    #     fig.legend()
    #     plt.grid()
    #     plt.xlabel("time [ms]")
    #     plt.ylabel("Vertical Force [N]")
    #     # fig.set_xlim(-0.10, 0.17)
    #     # fig.set_ylim(-100.0, 20.0)
    
    #
    # Drop test DeltaZ(t)
    #
    # for i in range(nb_files):
    #     fig = plt.figure("Height error, k=" + str((i+1)*100) + "N/m")
    #     plt.plot(list_pos_error_height_sensor[i], label = list_labels[i])
    #     plt.plot(list_pos_error_height_sensor_cutoff[i], label = list_labels[i] + " cutoff")
    #     plt.plot(list_pos_error_height_sensor_filtered[i], label = list_labels[i] + " cutoff")
    #     fig.legend()
    #     plt.grid()
    #     plt.xlabel("time [ms]")
    #     plt.ylabel("Vertical Foot Displacement [m]")
    
    #
    # Drop test mean( Fz(DeltaZ) )
    #
    # for i in range(nb_files):
    #     x=np.array([])
    #     y=np.array([])
    #     for phase in range(2, len(list_phase[i]), 2):
    #         x = np.append(x, list_pos_error_height_sensor[i][list_phase[i][phase]:list_phase[i][phase+1]])
    #         y = np.append(y, list_Fz[i][list_phase[i][phase]:list_phase[i][phase+1]])  
    #     y = y[ x < 0.1 ]
    #     x = x[ x < 0.1 ]
    #     poly = polyfit(x, y, 1)
    #     print("i=",i," polyfit=", poly)
    #     k_data = poly['polynomial'][0]
    #     dx = np.linspace(np.min(0.0), np.max(x), num=2000, endpoint=True)
    #     f = poly['polynomial'][0] * dx + poly['polynomial'][1]
    #     fig = plt.figure("Test"+str(i))
    #     plt.plot(x,y)
    #     plt.plot(dx,f)

    #
    # Drop test Fz(DeltaZ), Fz_max, mean( Fz(DeltaZ) ), theoretical Fz(DeltaZ)
    #
    for i in range(nb_files):
        fig = plt.figure("Drop test, k=" + str((i+1)*100) + "N/m")
        
        # Compute and plot theoretical stiffness
        dx = np.linspace(0.0, np.max(list_pos_error_height_sensor[i]), num=2000, endpoint=True)
        f = (i+1)*100 * dx
        gain_color = 0.2
        my_color = (gain_color * 0.0, gain_color * 0.0, gain_color * 1.0)
        plt.plot(dx, f, label = "Theoretical stiffness, k=" + str((i+1)*100), color="forestgreen")
        
        # Compute and plot actual stiffness
        x=np.array([])
        y=np.array([])
        for phase in range(2, len(list_phase[i]), 2):
            x = np.append(x, list_pos_error_height_sensor[i][list_phase[i][phase]:list_phase[i][phase+1]])
            y = np.append(y, list_Fz[i][list_phase[i][phase]:list_phase[i][phase+1]])
        y = y[ x < 0.09 ]
        x = x[ x < 0.09 ]
        poly = polyfit(x, y, 1)
        poly['polynomial'][0]
        f = poly['polynomial'][0] * dx + poly['polynomial'][1]
        plt.plot(dx,f, label = "Actual stiffness, k={:.2f}".format(poly['polynomial'][0]), color="darkorange")
        
        # compute the cartesian upper bound
        x0 = 0.28 # Resting position of the stiffness controller
        i_max = 12.0 # max current
        tau_max = 12.0 * 9.0 * 0.025 # max joint torque
        leg_segment_length = 0.16 # leg segment length (equal for upper and lower leg)
        alpha = np.arcsin( (x0 - dx) / (2 * leg_segment_length) ) # half knee angle
        level_arm = leg_segment_length * np.cos(alpha)
        Fz_max = tau_max / level_arm
        plt.plot(dx, Fz_max, label = "Max force (N)", color="black")

        # plot actual data with phase
        for start, stop, line_style, color, label in zip(list_phase[i][0:len(list_phase[i])-1], list_phase[i][1:], line_styles[i], colors[i], labels[i]):
            # plt.plot(list_pos_error_height_sensor_cutoff[i][start:stop], list_Fz_cutoff[i][start:stop], label=label, color=color, linestyle=line_style)
            plt.plot(list_pos_error_height_sensor[i][start:stop+1], list_Fz[i][start:stop+1], label=label, color=color, linestyle=line_style)

        plt.xlim(left=-0.004)
        fig.legend()
        plt.grid()
        plt.xlabel("Vertical Foot Displacement [m]")
        plt.ylabel("Vertical Force [N]")


    #
    # Drop test Fz(t), Fz_max(t), K * DeltaZ (>0)
    #
    # for i in range(nb_files):
    #     # setup the figure
    #     k = (i+1)*100
    #     fig = plt.figure("drop_test_vs_time_k_" + str(k))
        
    #     # The time array
    #     nb_time_pt = list_pos_error_height_sensor[i].shape[0]
    #     time = np.linspace(0.0, nb_time_pt * 0.001, num=nb_time_pt, endpoint=False)

    #     # The perfect control
    #     fz_perfect_ctrl = list_pos_error_height_sensor[i] * k
    #     fz_perfect_ctrl[ fz_perfect_ctrl < 0.0 ] = 0.0
                
    #     # compute the cartesian upper bound
    #     x0 = 0.28 # Resting position of the stiffness controller
    #     i_max = 12.0 # max current
    #     tau_max = 12.0 * 9.0 * 0.025 # max joint torque
    #     leg_segment_length = 0.16 # leg segment length (equal for upper and lower leg)
    #     alpha = np.arcsin( (x0 - list_pos_error_height_sensor[i]) / (2 * leg_segment_length) ) # half knee angle
    #     level_arm = leg_segment_length * np.cos(alpha)
    #     Fz_max = tau_max / level_arm
    #     Fz_max[ Fz_max > 42 ] = np.nan

    #     # The plots
    #     plt.plot(time, Fz_max, label = "Fz max", color="black")
    #     plt.plot(time, fz_perfect_ctrl, label="Fz ideal")
    #     plt.plot(time, list_Fz[i], label="Fz")

    #     fig.legend()
    #     plt.grid()
    #     plt.xlabel("time [s]")
    #     plt.ylabel("Vertical Force [N]")
        

    plt.show()
