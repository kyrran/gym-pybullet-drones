import os
from datetime import datetime
from cycler import cycler
import numpy as np
import matplotlib.pyplot as plt

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

class Logger(object):
    def __init__(self,
                 logging_freq_hz: int,
                 output_folder: str="results",
                 num_drones: int=1,
                 duration_sec: int=0,
                 colab: bool=False,
                 ):
        self.COLAB = colab
        self.OUTPUT_FOLDER = output_folder
        if not os.path.exists(self.OUTPUT_FOLDER):
            os.mkdir(self.OUTPUT_FOLDER)
        self.LOGGING_FREQ_HZ = logging_freq_hz
        self.NUM_DRONES = num_drones
        self.PREALLOCATED_ARRAYS = False if duration_sec == 0 else True
        self.counters = np.zeros(num_drones)
        self.timestamps = np.zeros((num_drones, duration_sec * self.LOGGING_FREQ_HZ))
        self.states = np.zeros((num_drones, 16, duration_sec * self.LOGGING_FREQ_HZ))
        self.controls = np.zeros((num_drones, 12, duration_sec * self.LOGGING_FREQ_HZ))
        self.payload_positions = np.zeros((3, duration_sec * self.LOGGING_FREQ_HZ))
        self.position_info = []  # Added to store timestamp and position information
        self.payload_position_info = []  # Added to store timestamp and payload position information



    def log(self,
            drone: int,
            timestamp,
            state,
            control=np.zeros(12),
            payload_position=None
            ):
        if drone < 0 or drone >= self.NUM_DRONES or timestamp < 0 or len(state) != 20 or len(control) != 12:
            print("[ERROR] in Logger.log(), invalid data")
        current_counter = int(self.counters[drone])
        if current_counter >= self.timestamps.shape[1]:
            self.timestamps = np.concatenate((self.timestamps, np.zeros((self.NUM_DRONES, 1))), axis=1)
            self.states = np.concatenate((self.states, np.zeros((self.NUM_DRONES, 16, 1))), axis=2)
            self.controls = np.concatenate((self.controls, np.zeros((self.NUM_DRONES, 12, 1))), axis=2)
            self.payload_positions = np.concatenate((self.payload_positions, np.zeros((3, 1))), axis=1)
        elif not self.PREALLOCATED_ARRAYS and self.timestamps.shape[1] > current_counter:
            current_counter = self.timestamps.shape[1] - 1
        self.timestamps[drone, current_counter] = timestamp
        self.states[drone, :, current_counter] = np.hstack([state[0:3], state[10:13], state[7:10], state[13:20]])
        self.controls[drone, :, current_counter] = control
        if payload_position is not None:
            self.payload_positions[:, current_counter] = payload_position
        
            self.payload_position_info.append((timestamp, *payload_position))  # Store timestamp and payload position
        self.position_info.append((timestamp, state[0], state[1], state[2]))  # Store timestamp and position

        self.counters[drone] = current_counter + 1

    def save(self):
        with open(os.path.join(self.OUTPUT_FOLDER, "save-flight-" + datetime.now().strftime("%m.%d.%Y_%H.%M.%S") + ".npy"), 'wb') as out_file:
            np.savez(out_file, timestamps=self.timestamps, states=self.states, controls=self.controls, payload_positions=self.payload_positions)

    def save_as_csv(self,
                    comment: str=""
                    ):
        csv_dir = os.path.join(self.OUTPUT_FOLDER, "save-flight-" + comment + "-" + datetime.now().strftime("%m.%d.%Y_%H.%M.%S"))
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
            with open(csv_dir + "/payload_x.csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.payload_positions[0, :]])), delimiter=",")
            with open(csv_dir + "/payload_y.csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.payload_positions[1, :]])), delimiter=",")
            with open(csv_dir + "/payload_z.csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.payload_positions[2, :]])), delimiter=",")
            
            
        with open(csv_dir + "/position_info.csv", 'wb') as out_file:  # Save position_info to CSV
            np.savetxt(out_file, self.position_info, delimiter=",")
        with open(csv_dir + "/payload_position_info.csv", 'wb') as out_file:  # Save payload_position_info to CSV
            np.savetxt(out_file, self.payload_position_info, delimiter=",")

    def plot(self, pwm=False):
        plt.rc('axes', prop_cycle=(cycler('color', ['r', 'g', 'b', 'y']) + cycler('linestyle', ['-', '--', ':', '-.'])))
        fig, axs = plt.subplots(14, 2)  # Adjusted to 14 rows to include the combined XYZ plot for payload
        t = np.arange(0, self.timestamps.shape[1] / self.LOGGING_FREQ_HZ, 1 / self.LOGGING_FREQ_HZ)

        col = 0

        # row = 0
        # for j in range(self.NUM_DRONES):
        #     axs[row, col].plot(t, self.states[j, 0, :], label="drone_" + str(j))
        # axs[row, col].set_xlabel('time')
        # axs[row, col].set_ylabel('x (m)')

        # row = 1
        # for j in range(self.NUM_DRONES):
        #     axs[row, col].plot(t, self.states[j, 1, :], label="drone_" + str(j))
        # axs[row, col].set_xlabel('time')
        # axs[row, col].set_ylabel('y (m)')

        # row = 2
        # for j in range(self.NUM_DRONES):
        #     axs[row, col].plot(t, self.states[j, 2, :], label="drone_" + str(j))
        # axs[row, col].set_xlabel('time')
        # axs[row, col].set_ylabel('z (m)')

        row = 0  # Starting row for the combined XYZ plot
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 0, :], label="x_drone_" + str(j))
            axs[row, col].plot(t, self.states[j, 1, :], label="y_drone_" + str(j))
            axs[row, col].plot(t, self.states[j, 2, :], label="z_drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('position (m)')
        axs[row, col].legend(loc='upper right')

        row = 1
        axs[row, col].plot(t, self.payload_positions[0, :], label="payload_x")
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('x (m)')

        row = 2
        axs[row, col].plot(t, self.payload_positions[1, :], label="payload_y")
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('y (m)')

        row = 3
        axs[row, col].plot(t, self.payload_positions[2, :], label="payload_z")
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('z (m)')

        row = 4  # Combined XYZ plot for payload
        axs[row, col].plot(t, self.payload_positions[0, :], label="x_payload")
        axs[row, col].plot(t, self.payload_positions[1, :], label="y_payload")
        axs[row, col].plot(t, self.payload_positions[2, :], label="z_payload")
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('position (m)')
        axs[row, col].legend(loc='upper right')

        row = 8
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 6, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('r (rad)')

        row = 9
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 7, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('p (rad)')

        row = 10
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 8, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('y (rad)')

        row = 11
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 9, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('wx')

        row = 12
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 10, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('wy')

        col = 1

        row = 0
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 3, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('vx (m/s)')

        row = 1
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 4, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('vy (m/s)')

        row = 2
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 5, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('vz (m/s)')

        row = 3
        for j in range(self.NUM_DRONES):
            rdot = np.hstack([0, (self.states[j, 6, 1:] - self.states[j, 6, 0:-1]) * self.LOGGING_FREQ_HZ])
            axs[row, col].plot(t, rdot, label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('rdot (rad/s)')

        row = 4
        for j in range(self.NUM_DRONES):
            pdot = np.hstack([0, (self.states[j, 7, 1:] - self.states[j, 7, 0:-1]) * self.LOGGING_FREQ_HZ])
            axs[row, col].plot(t, pdot, label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('pdot (rad/s)')

        row = 5
        for j in range(self.NUM_DRONES):
            ydot = np.hstack([0, (self.states[j, 8, 1:] - self.states[j, 8, 0:-1]) * self.LOGGING_FREQ_HZ])
            axs[row, col].plot(t, ydot, label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('ydot (rad/s)')

        for j in range(self.NUM_DRONES):
            for i in range(12, 16):
                if pwm and j > 0:
                    self.states[j, i, :] = (self.states[j, i, :] - 4070.3) / 0.2685

        row = 6
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 12, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        if pwm:
            axs[row, col].set_ylabel('PWM0')
        else:
            axs[row, col].set_ylabel('RPM0')

        row = 7
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 13, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        if pwm:
            axs[row, col].set_ylabel('PWM1')
        else:
            axs[row, col].set_ylabel('RPM1')

        row = 8
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 14, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        if pwm:
            axs[row, col].set_ylabel('PWM2')
        else:
            axs[row, col].set_ylabel('RPM2')

        row = 9
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 15, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        if pwm:
            axs[row, col].set_ylabel('PWM3')

        for i in range(14):  # Adjusted to 14 rows
            for j in range(2):
                axs[i, j].grid(True)
                axs[i, j].legend(loc='upper right', frameon=True)
        fig.subplots_adjust(left=0.06,
                            bottom=0.05,
                            right=0.99,
                            top=0.98,
                            wspace=0.15,
                            hspace=0.0
                            )
        if self.COLAB:
            plt.savefig(os.path.join('results', 'output_figure.png'))
        else:
            plt.show()
