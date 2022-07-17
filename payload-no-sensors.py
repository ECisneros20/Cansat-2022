import time
import datetime as dt
import math
import numpy as np
import csv

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

    while (1):
        print('Flight mode!')
        MISSION_TIME = str(dt.datetime.utcnow().hour) + ':' + str(dt.datetime.utcnow().minute) + ':' + str(dt.datetime.utcnow().second) + '.' + str(dt.datetime.utcnow().microsecond)[0:2]
        MESSAGE = str(TEAM_ID) + ',' + str(MISSION_TIME) + ',' + str(PACKET_COUNT) + ',' + str(PACKET_TYPE) + ',' + str(TP_ALTITUDE) + ',' + str(TP_TEMP) + ',' + str(TP_VOLTAGE) + ',' + str(GYRO_R) + ',' + str(GYRO_P) + ',' + str(GYRO_Y) + ',' + str(ACCEL_R) + ',' + str(ACCEL_P) + ',' + str(ACCEL_Y) + ',' + str(MAG_R) + ',' + str(MAG_P) + ',' + str(MAG_Y) + ',' + str(POINTING_ERROR) + ',' + str(TP_SOFTWARE_STATE)
        save_csv(MESSAGE)
        PACKET_COUNT += 1
        