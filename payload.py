import time
import datetime as dt
import math
import numpy as np
import csv

# BMP280
from bmp280 import BMP280
from smbus import SMBus

# INA219
from ina219 import INA219
from ina219 import DeviceRangeError

# MPU9250
from imusensor.MPU9250 import MPU9250

# Basic Inicialization
TEAM_ID = '1085'
MISSION_TIME = '00:00:00.00' # UTC from clock
PACKET_COUNT = 0 # Count of delivered packets
PACKET_TYPE = 'T'
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

States = ['LAUNCH_WAIT', 'ASCENT', 'ROCKET_SEPARATION', 'DESCENT', 'TP_RELEASE', 'LANDED', 'SIMULATION']
st = 0

declination = -16.71

# Initialize BMP280
bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bus)
pressure_ground_level = 1013.25 # hPa
altitude_zero = 100 # m

# Initialize INA219
SHUNT_OHMS = 0.1
ina = INA219(SHUNT_OHMS)
ina.configure()

# Initialize IMU
address = 0x68
imu = MPU9250.MPU9250(bus, address)
imu.begin()

# Initialize camera
# camera = picamera.PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 30

def get_altitude_zero():
    global pressure_ground_level, altitude_zero

    pressure = bmp280.get_pressure()
    temperature = bmp280.get_temperature()

    altitude_zero = ( (pressure_ground_level/pressure) ** (1/5.257) - 1) * (temperature+273.15) / 0.0065

def BMP280():
    global TP_ALTITUDE, TP_TEMP, pressure_ground_level, altitude_zero

    pressure = bmp280.get_pressure()
    temperature = bmp280.get_temperature()
    
    alt = ( (pressure_ground_level/pressure) ** (1/5.257) - 1) * (temperature+273.15) / 0.0065 - altitude_zero
    
    TP_TEMP = round(temperature,1)
    TP_ALTITUDE = round(alt,1)

def INA219():
    global VOLTAGE
    TP_VOLTAGE = round(ina.voltage(),2)

def MPU9250():
    global GYRO_R, GYRO_P,GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y, MAG_R, MAG_P, MAG_Y, POINTING_ERROR

    imu.readSensor()
    imu.computeOrientation()
    MAG_R = round(imu.MagVals[0]/100,0)
    MAG_P = round(imu.MagVals[1]/100,0)
    MAG_Y = round(imu.MagVals[2]/100,0)
    GYRO_R = str(round(np.degrees(imu.GyroVals[0]),0))
    GYRO_P = str(round(np.degrees(imu.GyroVals[1]),0))
    GYRO_Y = str(round(np.degrees(imu.GyroVals[2]),0))
    ACCEL_R = str(round(imu.AccelVals[0],0))
    ACCEL_P = str(round(imu.AccelVals[1],0))
    ACCEL_Y = str(round(imu.AccelVals[2],0))
    angulo = np.degrees(math.atan2(MAG_P, MAG_R))
    angulo -= declination
    if angulo < 0:
        angulo += 360
    POINTING_ERROR = angulo - 180

def save_csv(data):
    # Set csv_name corresponding to each module
    value_chain = data.split(',')
    if value_chain[3] == 'C':
        csv_name = "Flight_1085_C"
    elif value_chain[3] == 'T':
        csv_name = "Flight_1085_T"
    path_csv = "./" + csv_name + ".csv"
    with open(path_csv, "a", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(value_chain)

if __name__ == "__main__":

    get_altitude_zero()
    
    while (1):
        print('Flight mode!')
        BMP280()
        MPU9250()
        INA219()
        MISSION_TIME = str(dt.datetime.utcnow().hour) + ':' + str(dt.datetime.utcnow().minute) + ':' + str(dt.datetime.utcnow().second) + '.' + str(dt.datetime.utcnow().microsecond)[0:2]
        MESSAGE = str(TEAM_ID) + ',' + str(MISSION_TIME) + ',' + str(PACKET_COUNT) + ',' + str(PACKET_TYPE) + ',' + str(TP_ALTITUDE) + ',' + str(TP_TEMP) + ',' + str(TP_VOLTAGE) + ',' + str(GYRO_R) + ',' + str(GYRO_P) + ',' + str(GYRO_Y) + ',' + str(ACCEL_R) + ',' + str(ACCEL_P) + ',' + str(ACCEL_Y) + ',' + str(MAG_R) + ',' + str(MAG_P) + ',' + str(MAG_Y) + ',' + str(POINTING_ERROR) + ',' + str(TP_SOFTWARE_STATE)
        save_csv(MESSAGE)
        PACKET_COUNT += 1
        