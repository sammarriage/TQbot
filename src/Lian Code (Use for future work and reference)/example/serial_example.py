import serial
import time
from serial import SerialException
import serial.tools.list_ports


def open_close():
    # port_list = list(serial.tools.list_ports.comports())

    # if len(port_list) <= 0:
    #     print("No usage of serial ports")
    # for i in range(len(port_list)):
    #     print(port_list[i])

    s = serial.Serial('/dev/ttyCH341USB0', 9600)
    s.close()
    b = serial.Serial('/dev/ttyCH341USB0', 9600)
    b.close()

def read_example():
    try:
        s = serial.Serial('/dev/ttyCH341USB0', 9600)
        print("open ttyCH341USB0")
        # s.close()
    except SerialException as e:
        print("COM4 already open\n", e)

    print("sleeping")
    # time.sleep(3)
    print("sleeping finished")
    raw_data = s.readline()
    print("got raw data")
    try:
        data = raw_data.decode()
    except:
        data = raw_data

    print("got decoded data")
    print(len(raw_data), "-raw data: ", raw_data, "data: ", data, "in_waiting: ", s.in_waiting)
    # s.flush()
    s.close()

def general_example():
    port_list = list(serial.tools.list_ports.comports())

    for i in range(len(port_list)):
        print(port_list[i])

    try:
        s = serial.Serial('/dev/ttyCH341USB0', 9600)
        print("open ttyCH341USB0")
        # s.close()
    except SerialException as e:
        print("COM4 already open\n", e)

    print("sleeping")
    time.sleep(3)
    print("sleeping finished")
    t = 0
    while True:
        # time.sleep(1)
        raw_data = s.readline()
        try:
            data = raw_data.decode()
        except:
            data = None
            print("Error")
        print(len(raw_data), "---data: ", data, "in_waiting: ", s.in_waiting, "times: ", t)
        t += 1
    # print("in waiting: ", s.in_waiting)
    s.close()
    print("closed ttyCH341USB0")

    # print("open serial port")
    # s = serial.Serial('COM4')
    # time.sleep(3)
    # print(s.is_open)
    # for i in range(1000):
    #     raw_data = s.readline()
    #     try:
    #         data = raw_data.decode()
    #     except:
    #         data = None
    #         print("Error")
    #     print(len(raw_data), "---data: ", data, "in_waiting: ", s.in_waiting)

if __name__ == "__main__":
    read_example()
    # open_close()