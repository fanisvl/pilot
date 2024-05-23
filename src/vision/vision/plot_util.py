import matplotlib.pyplot as plt
import numpy as np
from numpy.random import rand

# Input: data_values, [2 : Accelerometer, Gyroscope][3 : x, y, z] 2D array of floats for each axis containing sensor measurements
# time_values: 1D array floats time in seconds time_values relative to the 1st measurement of data_values
# eg 0th element of data_values at time = 0s, 2nd at 0 + dt etc...
def plot_imu_data(data_values, time_values):

    figure, axis = plt.subplots(2, 1)
    figure.suptitle("IMU Measurements")

    # data for accelerometer
    axis[0].plot(time_values, data_values[0][0], "-r", label="x")
    axis[0].plot(time_values, data_values[0][1], "-g", label="y")
    axis[0].plot(time_values, data_values[0][2], "-b", label="z")
    axis[0].set_title("Accelerometer")
    axis[0].set_ylabel('Raw Data(m/s^2)')
    axis[0].legend(loc="lower left", prop={'size' : 10})
    axis[0].set_xlim(0, time_values[-1])
    axis[0].set_ylim(-4.5, 4.5)

    #data for gyroscope
    axis[1].plot(time_values, data_values[1][0], "-r", label="x")
    axis[1].plot(time_values, data_values[1][1], "-g", label="y")
    axis[1].plot(time_values, data_values[1][2], "-b", label="z")
    axis[1].set_title("Gyroscope")
    axis[1].set_ylabel('Raw Data(rad/s)')
    axis[1].set_xlabel("Time(s)")
    axis[1].legend(loc="lower left", prop={'size' : 10})
    axis[1].set_xlim(0, time_values[-1])
    axis[1].set_ylim(-4.5, 4.5)

    plt.subplots_adjust(hspace=0.7)
    plt.show()


#x = np.linspace(0, 1499, 1500)

#figure, axis = plt.subplots(3, 1)
#figure.suptitle("Sensor Measurements")


#axis[0].plot(x, np.array(rand((1500)) * 2.0 - 1.0), "-r", label="x")
#axis[0].plot(x, np.array(rand((1500)) * 2.0 - 1.0), "-g", label="y")
#axis[0].plot(x, np.array(rand((1500)) * 2.0 - 1.0), "-b", label="z")
#axis[0].set_title("Position")
#axis[0].set_ylabel('Raw Data(m)')
#axis[0].legend(loc="lower left", prop={'size' : 10})
#axis[0].set_xlim(0, 1500)
#axis[0].set_ylim(-4.5, 1.5)

#axis[1].plot(x, np.array(rand((1500)) * 2.0 - 1.0), "-r", label="x")
#axis[1].plot(x, np.array(rand((1500)) * 2.0 - 1.0), "-g", label="y")
#axis[1].plot(x, np.array(rand((1500)) * 2.0 - 1.0), "-b", label="z")
#axis[1].set_title("Rotation")
#axis[1].set_ylabel('Raw Data(rad)')
#axis[1].legend(loc="lower left", prop={'size' : 10})
#axis[1].set_xlim(0, 1500)
#axis[1].set_ylim(-4.5, 1.5)

#axis[2].plot(x, np.array(rand((1500)) * 2.0 - 1.0), "-r", label="x")
#axis[2].plot(x, np.array(rand((1500)) * 2.0 - 1.0), "-g", label="y")
#axis[2].plot(x, np.array(rand((1500)) * 2.0 - 1.0), "-b", label="z")
#axis[2].set_title("Acceleration")
#axis[2].set_ylabel('Raw Data(m/s^2)')
#axis[2].set_xlabel("Time(s)")
#axis[2].legend(loc="lower left", prop={'size' : 10})
#axis[2].set_xlim(0, 1500)
#axis[2].set_ylim(-4.5, 1.5)

#plt.subplots_adjust(hspace=0.7)
#plt.show()