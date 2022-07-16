import time
from time import sleep
import math
import numpy
import csv

# BMP280
from bmp280 import BMP280
from smbus import SMBus

# MPU9250
from imusensor.MPU9250 import MPU9250

# INA219
from ina219 import INA219
from ina219 import DeviceRangeError


try:
    telemetry = "OFF"

    with open('Flight_1085_T.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        data = csv_file.readlines()
        row = data[-1]
        TEAM_ID, MISSION_TIME, PACKET_COUNT, PACKET_TYPE, TP_ALTITUDE, TP_TEMP, TP_VOLTAGE, GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y, MAG_R, MAG_P, MAG_Y, POINTING_ERROR, TP_SOFTWARE_STATE = row.split(',')

    States = ['LAUNCH_WAIT', 'ASCENT', 'ROCKET_SEPARATION', 'DESCENT', 'TP_RELEASE', 'LANDED', 'SIMULATION']
    st = States.index(SOFTWARE_STATE)
    
    if st == 4 or st == 5:
        telemetry_TP = 'ON'
    else:
        telemetry_TP = 'OFF'
    #print(telemetry, telemetry_TP)

except:
    # Basic Inicialization
    TEAM_ID = '1085'
    MISSION_TIME = '00:00:00.00' # UTC from clock
    PACKET_COUNT = 0 # Count of delivered packets
    PACKET_TYPE = 'P'
    # BMP280
    TP_ALTITUDE = 0.0 # Resolution 0.1 m / from ground level
    # BMP280
    TP_TEMP = 0.0 # Resolution 0.1 °C
    # INA219
    TP_VOLTAGE = 0.00 # Resolution 0.01 V
    # MPU9250
    GYRO_R = 0
    GYRO_P = 0
    GYRO_Y = 0
    ACCEL_R = 0
    ACCEL_P = 0
    ACCEL_Y = 0
    MAG_R = 0
    MAG_P = 0
    MAG_Y = 0
    POINTING_ERROR = 0
    TP_SOFTWARE_STATE = 'LAUNCH_WAIT' # LAUNCH_WAIT / ASCENT / ROCKET_SEPARATION / DESCENT / TP_RELEASE / LANDED / SIMULATION

    # TEAM_ID + MISSION_TIME + PACKET_COUNT + PACKET_TYPE + TP_ALTITUDE + TP_TEMP + TP_VOLTAGE + GYRO_R + GYRO_P + GYRO_Y + ACCEL_R + ACCEL_P + ACCEL_Y + MAG_R + MAG_P + MAG_Y + POINTING_ERROR + TP_SOFTWARE_STATE
    MESSAGE_TP = ''

    States = ['LAUNCH_WAIT', 'ASCENT', 'ROCKET_SEPARATION', 'DESCENT', 'TP_RELEASE', 'LANDED', 'SIMULATION']
    st = 0
    telemetry_TP = 'OFF'

declination = -16.71

# Initialize XBEE S3B
PORT = "/dev/ttyAMA0" # CONTAINER
BAUD_RATE = 2400
ADDR_C = "0013A20041983FF3"

# Initialize BMP280
bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bus)
pressure_ground_level = 1013.25 # hPa ACTUALIZAR
altitude_zero = 635 # m
pressure_simulation = 0

# Initialize IMU
address = 0x68
imu = MPU9250.MPU9250(bus, address)
imu.begin()

# Initialize INA219
SHUNT_OHMS = 0.1
ina = INA219(SHUNT_OHMS)
ina.configure()

# Initialize camera
# camera = picamera.PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 30

def XBEE():
    global PORT, BAUD_RATE, device
    device = XBeeDevice(PORT, BAUD_RATE)
    device.open()

def get_altitude_zero():
    global pressure_ground_level, altitude_zero

    pressure = bmp280.get_pressure()
    temperature = bmp280.get_temperature()

    altitude_zero = ( (pressure_ground_level/pressure) ** (1/5.257) - 1) * (temperature+273.15) / 0.0065

def BMP280():
    global ALTITUDE, TEMP, pressure_ground_level, altitude_zero

    pressure = bmp280.get_pressure()
    temperature = bmp280.get_temperature()
    
    alt = ( (pressure_ground_level/pressure) ** (1/5.257) - 1) * (temperature+273.15) / 0.0065 - altitude_zero
    
    TEMP = round(temperature,1)
    ALTITUDE = round(alt,1)

def BMP280_simulation():
    global ALTITUDE, TEMP, pressure_ground_level, altitude_zero, pressure_simulation

    temperature = bmp280.get_temperature()
    
    alt = ( (pressure_ground_level/pressure_simulation) ** (1/5.257) - 1) * (temperature+273.15) / 0.0065 - altitude_zero
    
    TEMP = round(temperature,1)
    ALTITUDE = round(alt,1)

def INA219():
    global VOLTAGE
    VOLTAGE = round(ina.voltage(),2)

def MPU9250():
    global GYRO_R, GYRO_P,GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y, MAG_R, MAG_P, MAG_Y, POINTING_ERROR

    imu.readSensor()
    imu.computeOrientation()
    MAG_R = round(imu.MagVals[0]/100,0)
    MAG_P = round(imu.MagVals[1]/100,0)
    MAG_Y = round(imu.MagVals[2]/100,0)
    GYRO_R = str(round(numpy.degrees(imu.GyroVals[0]),0))
    GYRO_P = str(round(numpy.degrees(imu.GyroVals[1]),0))
    GYRO_Y = str(round(numpy.degrees(imu.GyroVals[2]),0))
    ACCEL_R = str(round(imu.AccelVals[0],0))
    ACCEL_P = str(round(imu.AccelVals[1],0))
    ACCEL_Y = str(round(imu.AccelVals[2],0))
    angulo = numpy.degrees(math.atan2(MAG_P, MAG_R))
    angulo -= declination
    if angulo < 0:
        angulo += 360
    POINTING_ERROR = angulo - 180

def dec2deg(value):
    dec = value/100.00
    deg = int(dec)
    min = (dec - int(dec))/0.6
    position = deg + min
    position = "%.4f" %(position)
    return position

def launch_wait():
    print('Launch wait started!')

    global st, TP_ALTITUDE

    while TP_ALTITUDE <= 10 and st == 0:
        pass
    st = 1

    print('Launch wait completed!')

def ascent():
    print('Ascent started!')

    global st, TP_ALTITUDE

    temporal = TP_ALTITUDE

    while TP_ALTITUDE - temporal >= 0:
        pass
    st = 2

    print('Ascent completed!')

def rocket_separation():
    print('Rocket separation started!')

    global st

    temporal = time.time_ns()

    while time.time_ns() - temporal < 3000000000:
        pass
    st = 3

    print('Rocket separation completed!')

def descent():
    print('Descent started!')

    global st, TP_ALTITUDE

    while TP_ALTITUDE > 300:
        pass

    st = 4

    print('Descent completed!')

def tp_release():
    print('TP release started!')

    global st, telemetry_TP

    telemetry_TP = 'ON'

    temporal = time.time()

    while time.time() - temporal < 5:
        pass

    while TP_ALTITUDE > 1:
        pass

    st = 5

    print('TP release completed!')

def landed():
    print('Process finished!')

# Lee mensajes del C
def readContainer():
    global device, ADDR_C, CMD_ECHO, MISSION_TIME, MODE, SOFTWARE_STATE, ALTITUDE, pressure_ground_level, TEMP, pressure_simulation, SIM_ENABLE, SIM_ACTIVATE, SIM_DISABLE, telemetry

    try:
        # Instantiate a remote XBee object.
        remote_C = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string(ADDR_C))

        device.flush_queues()

        print("Waiting for data...\n")
        start_time = time.time_ns()

        while True:
            if (time.time_ns() - start_time) > 1000000000:
                xbee_message = device.read_data_from(remote_C)
                start_time = time.time_ns()
                if xbee_message is not None:
                    data = xbee_message.data.decode()
                    command = data.split(',')
                    print("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                        xbee_message.data.decode()))
                    if command[0] == "CMD" and command[1] == "1085":
                        CMD_ECHO = command[2] + command[3]
                        if command[2] == "CX":
                            if command[3] == "ON" and telemetry == 'OFF':
                                telemetry = "ON"
                            elif command[3] == "OFF" and telemetry == "ON":
                                telemetry = "OFF"
                            else:
                                print("Invalid command!")
                        elif command[2] == "ST":
                            MISSION_TIME = command[3] + '.00'
                            print("Time set!")
                        elif command[2] == "SIM":
                            if command[3] == "ENABLE":
                                SIM_ENABLE = 1
                            elif command[3] == "ACTIVATE" and SIM_ENABLE == 1:
                                SIM_ACTIVATE = 1
                                MODE = 'S'
                                st = 6
                                SOFTWARE_STATE = States[st]
                            elif command[3] == "DISABLE":
                                SIM_DISABLE = 1
                                SIM_ENABLE = 0
                                SIM_ACTIVATE = 0
                                MODE = 'F'
                                st = 0
                                SOFTWARE_STATE = States[st]
                            else:
                                print("Invalid command!")
                        elif command[2] == 'SIMP':
                            pressure_simulation = command[3]
                        else:
                            print("Invalid command!")
                    else:
                        print("Invalid command!")

    finally:
        if device is not None and device.is_open():
            device.close()

# Medir los tres sensores necesarios
def measure_telemetry():
    global telemetry, MODE, device, ADDR_GS, MESSAGE, PACKET_COUNT, TEAM_ID, MISSION_TIME, PACKET_COUNT, PACKET_TYPE, ALTITUDE, TEMP, VOLTAGE, GPS_TIME, GPS_LATITUDE, GPS_LONGITUDE, GPS_ALTITUDE, GPS_SATS, SOFTWARE_STATE, CMD_ECHO
    
    try:
        #print(telemetry)
        # Instantiate a remote XBee object.
        remote_C = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string(ADDR_C))

        device.flush_queues()

        print("Ready to send data...\n")
        start_time = time.time_ns()

        while True:
            if telemetry == "ON":
                if MODE == 'F':
                    BMP280()
                    INA219()
                    MPU9250()
                    SOFTWARE_STATE = States[st] #Cambia según el estado donde se encuentre
                elif MODE == 'S':
                    BMP280_simulation()
                    INA219()
                    MPU9250()
                    SOFTWARE_STATE = States[st] #Cambia según el estado donde se encuentre

                if (time.time_ns() - start_time) > 250000000:
                    MESSAGE = str(TEAM_ID) + ',' + str(MISSION_TIME) + ',' + str(PACKET_COUNT) + ',' + str(PACKET_TYPE) + ',' + str(ALTITUDE) + ',' + str(TEMP) + ',' + str(VOLTAGE) + ',' + str(GYRO_R) + ',' + str(GYRO_P) + ',' + str(GYRO_Y) + ',' + str(ACCEL_R) + ',' + str(ACCEL_P) + ',' + str(ACCEL_Y) + ','+ str(MAG_R) + ','+ str(MAG_P) + ','+ str(MAG_Y) + ','+ str(POINTING_ERROR) + ','+ str(SOFTWARE_STATE)
                    guardarData(MESSAGE)
                    device.send_data(remote_C, MESSAGE)
                    start_time = time.time_ns()
                    print("Sent to %s >> %s" % (ADDR_C,MESSAGE))
                    PACKET_COUNT = PACKET_COUNT + 1
    finally:
        if device is not None and device.is_open():
            device.close()

def guardarData(data):
    # Set csv_name corresponding to each module
    value_chain = data.split(',')
    csv_name = "Flight_1085_T_"
    path_csv = csv_name +  ".csv"
    with open(path_csv,"a",newline='') as f:
        writer = csv.writer(f)#, delimiter=",")
        writer.writerow(value_chain)

def flight_mode():
    launch_wait()
    ascent()
    rocket_separation()
    descent()
    tp_release()
    landed()

def simulation_mode():
    launch_wait()
    ascent()
    rocket_separation()
    descent()
    tp_release()
    landed()

if __name__ == "__main__":

    XBEE()

    _thread.start_new_thread(readContainer,())
    _thread.start_new_thread(measure_telemetry,())
    
    while (1):
        if SIM_DISABLE:
            print('Flight mode!')
            get_altitude_zero()
            flight_mode()
        elif SIM_ENABLE and SIM_ACTIVATE:
            print('Simulation mode!')
            get_altitude_zero()
            simulation_mode()
        else:
            print('Select a mode!')