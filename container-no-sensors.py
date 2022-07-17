import time
import datetime as dt
import csv

# Basic Inicialization
TEAM_ID = '1085'
MISSION_TIME = '00:00:00.00' # UTC from clock
PACKET_COUNT = 0 # Count of delivered packets
PACKET_TYPE = 'C'
MODE = 'F' # F default / S
TP_RELEASED = 'N' # N / R
# BMP280
ALTITUDE = 0.0 # Resolution 0.1 m / from ground level
# BMP280
TEMP = 0.0 # Resolution 0.1 °C
# INA219
VOLTAGE = 0.00 # Resolution 0.01 V
# NEOM8N
GPS_TIME = '00:00:00' # UTC from clock
GPS_LATITUDE = '0.0000' # Resolution 0.0001 decimal degrees North
GPS_LONGITUDE = '0.0000' # Resolution 0.0001 decimal degrees West
GPS_ALTITUDE = '0.0' # Resolution 0.1 meters
GPS_SATS = 0 # Number of GPS satellites being tracked by the GPS receiver
SOFTWARE_STATE = 'LAUNCH_WAIT' # LAUNCH_WAIT / ASCENT / ROCKET_SEPARATION / DESCENT / TP_RELEASE / LANDED / SIMULATION
CMD_ECHO = '' # Command like CXON / ST101325
# TEAM_ID + MISSION_TIME + PACKET_COUNT + PACKET_TYPE + MODE + TP_RELEASED + ALTITUDE + TEMP + VOLTAGE + GPS_TIME + GPS_LATITUDE + GPS_LONGITUDE + GPS_ALTITUDE + GPS_SATS + SOFTWARE_STATE + CMD_ECHO

States = ['LAUNCH_WAIT', 'ASCENT', 'ROCKET_SEPARATION', 'DESCENT', 'TP_RELEASE', 'LANDED', 'SIMULATION']
st = 0

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
        MESSAGE = str(TEAM_ID) + ',' + str(MISSION_TIME) + ',' + str(PACKET_COUNT) + ',' + str(PACKET_TYPE) + ',' + str(MODE) + ',' + str(TP_RELEASED) + ',' + str(ALTITUDE) + ',' + str(TEMP) + ',' + str(VOLTAGE) + ',' + str(GPS_TIME) + ',' + str(GPS_LATITUDE) + ',' + str(GPS_LONGITUDE) + ',' + str(GPS_ALTITUDE) + ',' + str(GPS_SATS) + ',' + str(SOFTWARE_STATE) + ',' + str(CMD_ECHO)
        save_csv(MESSAGE)
        PACKET_COUNT += 1
        