import os
import time
from datetime import datetime
from cycler import cycler
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import csv

os.environ['KMP_DUPLICATE_LIB_OK']='True'

class Logger(object):
    """A class for logging and visualization.

    Stores, saves to file, and plots the kinematic information and RPMs
    of a simulation with one or more drones.

    """

    ################################################################################

    def __init__(self,
                 logging_freq_hz: int,
                 num_drones: int=1,
                 duration_sec: int=0
                 ):
        """Logger class __init__ method.

        Parameters
        ----------
        logging_freq_hz : int
            Logging frequency in Hz.
        num_drones : int, optional
            Number of drones.
        duration_sec : int, optional
            Used to preallocate the log arrays (improves performance).

        """
        self.LOGGING_FREQ_HZ = logging_freq_hz
        self.NUM_DRONES = num_drones
        self.PREALLOCATED_ARRAYS = False if duration_sec == 0 else True
        self.counters = np.zeros(num_drones)
        self.timestamps = np.zeros((num_drones, duration_sec*self.LOGGING_FREQ_HZ))
        self.allstates = np.zeros((num_drones, 20, duration_sec*self.LOGGING_FREQ_HZ))
        self.states = np.zeros((num_drones, 16, duration_sec*self.LOGGING_FREQ_HZ)) #### 16 states: pos_x,
                                                                                                  # pos_y,
                                                                                                  # pos_z,
                                                                                                  # vel_x,
                                                                                                  # vel_y,
                                                                                                  # vel_z,
                                                                                                  # roll,
                                                                                                  # pitch,
                                                                                                  # yaw,
                                                                                                  # ang_vel_x,
                                                                                                  # ang_vel_y,
                                                                                                  # ang_vel_z,
                                                                                                  # rpm0,
                                                                                                  # rpm1,
                                                                                                  # rpm2,
                                                                                                  # rpm3
        #### Note: this is not the same order nor length ###########
        self.controls = np.zeros((num_drones, 12, duration_sec*self.LOGGING_FREQ_HZ)) #### 12 control targets: pos_x,
                                                                                                             # pos_y,
                                                                                                             # pos_z,
                                                                                                             # vel_x, 
                                                                                                             # vel_y,
                                                                                                             # vel_z,
                                                                                                             # roll,
                                                                                                             # pitch,
                                                                                                             # yaw,
                                                                                                             # ang_vel_x,
                                                                                                             # ang_vel_y,
                                                                                                             # ang_vel_z

    ################################################################################

    def log(self,
            drone: int,
            timestamp,
            state,
            control=np.zeros(12)
            ):
        """Logs entries for a single simulation step, of a single drone.

        Parameters
        ----------
        drone : int
            Id of the drone associated to the log entry.
        timestamp : float
            Timestamp of the log in simulation clock.
        state : ndarray
            (20,)-shaped array of floats containing the drone's state.
        control : ndarray, optional
            (12,)-shaped array of floats containing the drone's control target.

        """
        if drone < 0 or drone >= self.NUM_DRONES or timestamp < 0 or len(state) != 20 or len(control) != 12:
            print("[ERROR] in Logger.log(), invalid data")
        current_counter = int(self.counters[drone])
        #### Add rows to the matrices if a counter exceeds their size
        if current_counter >= self.timestamps.shape[1]:
            self.timestamps = np.concatenate((self.timestamps, np.zeros((self.NUM_DRONES, 1))), axis=1)
            self.states = np.concatenate((self.states, np.zeros((self.NUM_DRONES, 16, 1))), axis=2)
            self.allstates = np.concatenate((self.allstates, np.zeros((self.NUM_DRONES, 20, 1))), axis=2)
            self.controls = np.concatenate((self.controls, np.zeros((self.NUM_DRONES, 12, 1))), axis=2)
        #### Advance a counter is the matrices have overgrown it ###
        elif not self.PREALLOCATED_ARRAYS and self.timestamps.shape[1] > current_counter:
            current_counter = self.timestamps.shape[1]-1
        #### Log the information and increase the counter ##########
        self.timestamps[drone, current_counter] = timestamp
        self.states[drone, :, current_counter] = np.hstack([state[0:3], state[10:13], state[7:10], state[13:20]])
        self.allstates[drone, :, current_counter] = state
        self.allstates[drone, 10:13, current_counter] = p.getEulerFromQuaternion(self.allstates[drone, 3:7, current_counter])
        self.controls[drone, :, current_counter] = control
        self.counters[drone] = current_counter + 1

    ################################################################################

    def save_as_csv_2(self,
                    comment: str = ""
                    ):
        """Save the logs---on your Desktop---as comma separated values.
        Parameters
        ----------
        comment : str, optional
            Added to the foldername.
        """
        csv_dir = "../files/logs/"
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir + '/')
        t = np.arange(0, self.timestamps.shape[1] / self.LOGGING_FREQ_HZ, 1 / self.LOGGING_FREQ_HZ)
        for i in range(self.NUM_DRONES):
            with open(csv_dir + "/x" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 0, :]])), delimiter=",")
            with open(csv_dir + "/y" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 1, :]])), delimiter=",")
            with open(csv_dir + "/z" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 2, :]])), delimiter=",")
            ####
            with open(csv_dir + "/r" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 6, :]])), delimiter=",")
            with open(csv_dir + "/p" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 7, :]])), delimiter=",")
            with open(csv_dir + "/ya" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 8, :]])), delimiter=",")
            ####
            with open(csv_dir + "/rr" + str(i) + ".csv", 'wb') as out_file:
                rdot = np.hstack([0, (self.states[i, 6, 1:] - self.states[i, 6, 0:-1]) * self.LOGGING_FREQ_HZ])
                np.savetxt(out_file, np.transpose(np.vstack([t, rdot])), delimiter=",")
            with open(csv_dir + "/pr" + str(i) + ".csv", 'wb') as out_file:
                pdot = np.hstack([0, (self.states[i, 7, 1:] - self.states[i, 7, 0:-1]) * self.LOGGING_FREQ_HZ])
                np.savetxt(out_file, np.transpose(np.vstack([t, pdot])), delimiter=",")
            with open(csv_dir + "/yar" + str(i) + ".csv", 'wb') as out_file:
                ydot = np.hstack([0, (self.states[i, 8, 1:] - self.states[i, 8, 0:-1]) * self.LOGGING_FREQ_HZ])
                np.savetxt(out_file, np.transpose(np.vstack([t, ydot])), delimiter=",")
            ###
            with open(csv_dir + "/vx" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 3, :]])), delimiter=",")
            with open(csv_dir + "/vy" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 4, :]])), delimiter=",")
            with open(csv_dir + "/vz" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 5, :]])), delimiter=",")
            ####
            with open(csv_dir + "/wx" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 9, :]])), delimiter=",")
            with open(csv_dir + "/wy" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 10, :]])), delimiter=",")
            with open(csv_dir + "/wz" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 11, :]])), delimiter=",")
            ####
            with open(csv_dir + "/rpm0-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 12, :]])), delimiter=",")
            with open(csv_dir + "/rpm1-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 13, :]])), delimiter=",")
            with open(csv_dir + "/rpm2-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 14, :]])), delimiter=",")
            with open(csv_dir + "/rpm3-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 15, :]])), delimiter=",")
            ####
            with open(csv_dir + "/pwm0-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, (self.states[i, 12, :] - 4070.3) / 0.2685])),
                           delimiter=",")
            with open(csv_dir + "/pwm1-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, (self.states[i, 13, :] - 4070.3) / 0.2685])),
                           delimiter=",")
            with open(csv_dir + "/pwm2-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, (self.states[i, 14, :] - 4070.3) / 0.2685])),
                           delimiter=",")
            with open(csv_dir + "/pwm3-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, (self.states[i, 15, :] - 4070.3) / 0.2685])),
                           delimiter=",")

    def save_as_csv(self,
                    comment: str = ""
                    ):
        """Save the logs---on your Desktop---as comma separated values.
        Parameters
        ----------
        comment : str, optional
            Added to the foldername.
        """
        csv_dir = "../files/logs/"
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir + '/')
        t = np.arange(0, self.timestamps.shape[1] / self.LOGGING_FREQ_HZ, 1 / self.LOGGING_FREQ_HZ)
        for i in range(self.NUM_DRONES):
            with open(csv_dir + "/x" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.allstates[i, 0, :]])), delimiter=",")
            with open(csv_dir + "/y" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.allstates[i, 1, :]])), delimiter=",")
            with open(csv_dir + "/z" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.allstates[i, 2, :]])), delimiter=",")
            with open(csv_dir + "/q0" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.allstates[i, 3, :]])), delimiter=",")
            with open(csv_dir + "/q1" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.allstates[i, 4, :]])), delimiter=",")
            with open(csv_dir + "/q2" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.allstates[i, 5, :]])), delimiter=",")
            with open(csv_dir + "/q3" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.allstates[i, 6, :]])), delimiter=",")


    def save(self):
        """Save the logs to file.
        """
        # with open('C:/Users/Peter/Documents/sztaki/pilcoV0.9/scenarios/crazyflie/params.csv', newline='') as paramfile:
        #     reader = csv.reader(paramfile)
        #     params = list(reader)
        #     params = np.array(params)
        # if len(params) < 2:
        #     out_filename = 'random-trial-' + str(int(params))
        # else:
        #     out_filename = 'controlled-trial-' + str(int(params[-1, -1]))
        out_filename = "save-flight-" + datetime.now().strftime("%m.%d.%Y_%H.%M.%S")
        with open(os.path.dirname(os.path.abspath(__file__))+"/../../files/logs/save-flight-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+".csv", 'wb') as out_file:
            #  np.savetxt(out_file, self.timestamps)

            for j in range(self.NUM_DRONES):
                np.savetxt(out_file, self.allstates[j, :, :], delimiter=',')

    # def save(self):
    #     """Save the logs to file.
    #     """
    #     with open(os.path.dirname(os.path.abspath(__file__))+"/../../files/logs/save-flight-"+datetime.now().strftime("%m.%d.%Y_%H.%M.%S")+".npy", 'wb') as out_file:
    #         np.savez(out_file, timestamps=self.timestamps, states=self.states, controls=self.controls, allstates=self.allstates)

    ################################################################################
    
    def plot(self, pwm=False):
        """Logs entries for a single simulation step, of a single drone.

        Parameters
        ----------
        pwm : bool, optional
            If True, converts logged RPM into PWM values (for Crazyflies).

        """
        #### Loop over colors and line styles ######################
        plt.rc('axes', prop_cycle=(cycler('color', ['r', 'g', 'b', 'y']) + cycler('linestyle', ['-', '--', ':', '-.'])))
        fig, axs = plt.subplots(8, 2)
        t = np.arange(0, self.timestamps.shape[1]/self.LOGGING_FREQ_HZ, 1/self.LOGGING_FREQ_HZ)
        if pwm:
            labels = ['X', 'Y', 'Z', 'VX', 'VY', 'VZ', 'R', 'P', 'Yaw', 'WR', 'WP', 'WY', 'PWM0', 'PWM1', 'PWM2', 'PWM3']
        else:
            labels = ['X', 'Y', 'Z', 'VX', 'VY', 'VZ', 'R', 'P', 'Yaw', 'WR', 'WP', 'WY', 'RPM0', 'RPM1', 'RPM2', 'RPM3']
        for i in range(16):
            for j in range(self.NUM_DRONES):
                #### This IF converts RPM into PWM for all drones ##########
                #### except drone_0 (only used in examples/compare.py) #####
                if pwm and i > 11 and j > 0:
                    self.states[j, i, :] = (self.states[j, i, :] - 4070.3) / 0.2685
                axs[i%8, i//8].plot(t, self.states[j, i, :], label="drone_"+str(j))
            axs[i%8, i//8].set_xlabel('time')
            axs[i%8, i//8].set_ylabel(labels[i])
            axs[i%8, i//8].grid(True)
            axs[i%8, i//8].legend(loc='upper right',
                                  frameon=True
                                  )
        axs[7, 0].plot(t, self.controls[0, 3, :])
        axs[0, 0].plot(t, self.controls[0, 0, :])
        axs[2, 0].plot(t, self.controls[0, 2, :])
        fig.subplots_adjust(left=0.06,
                            bottom=0.05,
                            right=0.99,
                            top=0.98,
                            wspace=0.15,
                            hspace=0.2
                            )
        plt.show()
