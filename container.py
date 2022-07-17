import time
import datetime as dt
import csv

# BMP280
from sensors.bmp280 import BMP280
from smbus import SMBus

# INA219
from sensors.ina219 import INA219
from sensors.ina219 import DeviceRangeError

# NEOM8N
import pigpio
import os

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

# Initialize BMP280
bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bus)
pressure_ground_level = 1013.25 # hPa
altitude_zero = 100 # m

# Initialize INA219
SHUNT_OHMS = 0.1
ina = INA219(SHUNT_OHMS)
ina.configure()

# Initialize NEOM8N
RX = 12

# # Initialize camera
# camera = picamera.PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 30

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

def INA219():
    global VOLTAGE
    VOLTAGE = round(ina.voltage(),2)

def dec2deg(value):
    dec = value/100.00
    deg = int(dec)
    min = (dec - int(dec))/0.6
    position = deg + min
    position = "%.4f" %(position)
    return position

def NEOM8N():
    global GPS_TIME, GPS_LATITUDE, GPS_LONGITUDE, GPS_ALTITUDE, GPS_SATS, RX

    try:
        # Create instance of pigpio class
        pi = pigpio.pi()
        if not pi.connected:
            os.system("sudo pigpiod")
            time.sleep(1)
            pi = pigpio.pi()
        pi.set_mode(RX, pigpio.INPUT)
        
        try:
            pi.bb_serial_read_open(RX, 9600, 8)
        except:
            pi.bb_serial_read_close(RX)
            pi.bb_serial_read_open(RX, 9600, 8)

        #print ("DATA - SOFTWARE SERIAL:")
        while (1):
            (count, data) = pi.bb_serial_read(RX)
            if count:
                gpsdata = data.decode("utf8",errors='replace')
                indice1 = gpsdata.find("GNGGA")
                indice2 = gpsdata.find("GNGSA")
                if indice1 !=- 1 and indice2 !=- 1:
                    linea = gpsdata[indice1:indice2-2]
                    datalinea = linea.split(',')
                    GPS_LATITUDE = round(dec2deg(float(datalinea[2])),4)
                    GPS_LONGITUDE = round(dec2deg(float(datalinea[4])),4)
                    hora = datalinea[1].split('.')
                    GPS_TIME = hora[0]
                    GPS_SATS = datalinea[7]
                    GPS_ALTITUDE = round(datalinea[9],1)
            time.sleep(0.5)

    except:
        pi.bb_serial_read_close(RX)
        pi.stop()

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
        INA219()
        NEOM8N()
        MISSION_TIME = str(dt.datetime.utcnow().hour) + ':' + str(dt.datetime.utcnow().minute) + ':' + str(dt.datetime.utcnow().second) + '.' + str(dt.datetime.utcnow().microsecond)[0:2]
        MESSAGE = str(TEAM_ID) + ',' + str(MISSION_TIME) + ',' + str(PACKET_COUNT) + ',' + str(PACKET_TYPE) + ',' + str(MODE) + ',' + str(TP_RELEASED) + ',' + str(ALTITUDE) + ',' + str(TEMP) + ',' + str(VOLTAGE) + ',' + str(GPS_TIME) + ',' + str(GPS_LATITUDE) + ',' + str(GPS_LONGITUDE) + ',' + str(GPS_ALTITUDE) + ',' + str(GPS_SATS) + ',' + str(SOFTWARE_STATE) + ',' + str(CMD_ECHO)
        save_csv(MESSAGE)
        PACKET_COUNT += 1
        