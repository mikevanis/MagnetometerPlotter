#!/usr/bin/env python

from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math

class SerialPlot:
    def __init__(self, serial_port='/dev/tty.NorthLight-ESP32SPP', serial_baud=115200, plotLength=100, dataNumBytes=2):
        self.port = serial_port
        self.baud = serial_baud
        self.plotMaxLength = plotLength
        self.dataNumBytes = dataNumBytes
        self.rawData = bytearray(dataNumBytes)
        self.mag_readings = []
        self.data = collections.deque([0] * plotLength, maxlen=plotLength)
        self.isRun = True
        self.isReceiving = False
        self.is_connected = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        # self.csvData = []

        print('Trying to connect to: ' + str(serial_port) + ' at ' + str(serial_baud) + ' BAUD.')
        while not self.is_connected:
            try:
                self.serialConnection = serial.Serial(serial_port, serial_baud, timeout=4)
                print('Connected to ' + str(serial_port) + ' at ' + str(serial_baud) + ' BAUD.')
                self.is_connected = True
            except serial.serialutil.SerialException:
                print("Failed to connect with " + str(serial_port) + ' at ' + str(serial_baud) + ' BAUD. Retrying in 5.')
                time.sleep(5)

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving is False:
                time.sleep(0.1)

    def getSerialData(self, i, scatter):
        scatter.set_offsets(self.mag_readings)
        return scatter,

    def backgroundThread(self):  # retrieve data
        time.sleep(0.2)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            line = self.serialConnection.readline()
            mag_readings = line.split(b',')
            if len(mag_readings) == 2:
                x_reading = float(mag_readings[0][1:])
                y_reading = float(mag_readings[1][1:])

                # Smooth values if array is big enough
                if len(self.mag_readings) > 2:
                    x_readings = [i[0] for i in self.mag_readings]
                    y_readings = [i[1] for i in self.mag_readings]
                    x_averaged = (sum(x_readings[-2:])+x_reading) / 3
                    y_averaged = (sum(y_readings[-2:])+y_reading) / 3
                    x_reading = x_averaged
                    y_reading = y_averaged

                self.mag_readings.append([x_reading, y_reading])

                try:
                    x_readings = [i[0] for i in self.mag_readings]
                    y_readings = [i[1] for i in self.mag_readings]
                    x_min = np.min(x_readings)
                    x_max = np.max(x_readings)
                    y_min = np.min(y_readings)
                    y_max = np.max(y_readings)
                    x_std = np.std(x_readings)
                    y_std = np.std(y_readings)
                    a = (x_max + x_min) / 2
                    b = (y_max + y_min) / 2
                    heading = (math.atan2(x_reading-a, y_reading-b) * 180) / math.pi
                    print("std_x: " + str(x_std) + "\tstd_y: " + str(y_std))
                    print("a: " + str(a) + "\tb: " + str(b))
                    print("Heading: " + str(heading))
                except IndexError:
                    print("Array not big enough to calculate heading.")
                #print('X:  {}\t Y:  {}'.format(x_reading, y_reading))
                self.isReceiving = True

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')


def main():
    s = SerialPlot()  # initializes all required variables
    s.readSerialStart()  # starts background thread

    # plotting starts below
    pltInterval = 200  # Period at which the plot animation updates [ms]
    xmin = -100
    xmax = 100
    ymin = -100
    ymax = 100
    fig = plt.figure()
    ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
    ax.set_title('Magnetometer readings')
    ax.set_xlabel("X (micro-Tesla)")
    ax.set_ylabel("Y (micro-Tesla)")
    scatter = ax.scatter([], [])

    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(scatter, ),
                                   blit=True, interval=pltInterval, frames=5000)  # fargs has to be a tuple

    plt.show()
    s.close()


if __name__ == '__main__':
    main()
