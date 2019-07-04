import sys, os
from copy import deepcopy
import numpy as np
from numpy import math
import RAI
import pkg_resources

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
def get_data(rai_data, start_value, end_value, index):
  """
  Parse the data file
  """

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
  pos_error_height_sensor_filtered = 0.27 - (pos_error_height_sensor_filtered - 0.01) 
  pos_error_height_sensor = 0.27 - (pos_error_height_sensor - 0.01)
  # 
  Fz = 0.0 + Fz
  Fz_filtered = 0.0 + Fz_filtered

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


def parse_args(argv):
    argc = len(argv)
    print ("argc =", argc)
    assert argc == 2
    data_path = sys.argv[1]
    dat_folders = [f for f in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, f))]
    dat_folders.sort()
    file_names = []
    list_labels = []
    for folder in dat_folders:
        file_names.append(os.path.join(#os.path.abspath(data_path),
                          folder, folder +  ".npz"))
    file_names.sort()
    nb_files = len(file_names)
    return nb_files, file_names, list_labels


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
    nb_files, file_names, list_labels = parse_args(sys.argv)
    print ("number of file to read: ", nb_files)
    print ("file names: ")
    for f in file_names:
      print (f)

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
    list_k_value_z_theo = []

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

        try:
          (Fx, Fz, Fz_filtered, pos_error_x, pos_error_z, pos_error_height_sensor,
          pos_error_height_sensor_filtered, HFE_u, KFE_u,
          k_value_x, k_value_z) = get_data(list_sensors_data[i], start_value, end_value, i)
          
          list_Fx.append(Fx)
          list_Fz.append(Fz)
          list_Fz_filtered.append(Fz_filtered)
          list_pos_error_x.append(pos_error_x)
          list_pos_error_z.append(pos_error_z)
          list_pos_error_height_sensor.append(pos_error_height_sensor)
          list_pos_error_height_sensor_filtered.append(pos_error_height_sensor_filtered)
          list_HFE_u.append(HFE_u)
          list_KFE_u.append(KFE_u)
          list_k_value_x.append(k_value_x)
          list_k_value_z.append(k_value_z)
        except KeyError as err:
          print("Key error: {0}".format(err))
        except:
          print("Unexpected error:", sys.exc_info()[0])
          pass

    #
    # Prepare plot
    #
    figure, subplot = plt.subplots(1,1, sharex = True)

    for i in range(len(list_Fx)):
        subplot.plot(list_pos_error_height_sensor_filtered[i], list_Fz_filtered[i], label = "K=" + str((i+1)*20) + "N/m")

    subplot.set_xlim(-0.05, 0.21)
    subplot.set_ylim(-21, 21)
    subplot.legend()
    subplot.set_xlabel("Vertical Foot Displacement [m]")
    subplot.set_ylabel("Vertical Force [N]")
    subplot.grid()

    plt.show()
