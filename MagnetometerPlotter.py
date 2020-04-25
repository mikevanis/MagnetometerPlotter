import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')
fig = plt.gcf()
fig.show()
plt.xlim([-1000, 1000])
plt.ylim([-1000, 1000])
fig.canvas.draw()
plt.title("Magnetometer readings")

x_readings = []
y_readings = []


def draw_data(x_data, y_data):
    print("Animation!")
    x_data = x_data[-40:]
    y_data = y_data[-40:]

    plt.scatter(x_data, y_data)

    plt.pause(0.2)
    fig.canvas.draw()


while True:
    try:
        with serial.Serial('/dev/tty.NorthLight-ESP32SPP', 115200, timeout=1) as ser:
            print("Connected!")
            while True:
                line = ser.readline()
                print(line)
                mag_readings = line.split(b',')
                print(mag_readings)
                if len(mag_readings) == 2:
                    x_reading = int(mag_readings[0][1:])
                    y_reading = int(mag_readings[1][1:])
                    x_readings.append(x_reading)
                    y_readings.append(y_reading)
                    draw_data(x_readings, y_readings)
                    print('X:  {}\t Y:  {}'.format(x_reading, y_reading))
    except serial.serialutil.SerialException:
        print("Could not connect to serial port. Retrying in 5 seconds...")
        time.sleep(5)
