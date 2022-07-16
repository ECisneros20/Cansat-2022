import time
from time import sleep
import math
import numpy
import csv

# BMP280
from bmp280 import BMP280
from smbus import SMBus

# INA219
from ina219 import INA219
from ina219 import DeviceRangeError

# NEOM8N
import pigpio
import os

# DS3231
import SDL_DS3231

# MG90S / 2N7000 / LED / Buzzer / motor / IRF530
import RPi.GPIO as GPIO

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
MESSAGE = ''
MESSAGE_TP = ''

States = ['LAUNCH_WAIT', 'ASCENT', 'ROCKET_SEPARATION', 'DESCENT', 'TP_RELEASE', 'LANDED', 'SIMULATION']
st = 0
telemetry = "OFF"
telemetry_TP = 'OFF'

# Initialize XBEE S3B
PORT = "/dev/ttyAMA0" # CONTAINER
BAUD_RATE = 1200
ADDR_GS = "0013A20041983FCF" # GROUND STATION
ADDR_PAYLOAD = "0013A200419842C2" # PAYLOAD
SIM_ENABLE = 0
SIM_ACTIVATE = 0
SIM_DISABLE = 0

# Initialize BMP280
bus = SMBus(1)
bmp280 = BMP280(i2c_dev=bus)
pressure_ground_level = 1013.25 # hPa ACTUALIZAR
altitude_zero = 635 # m
pressure_simulation = 0

# Initialize INA219
SHUNT_OHMS = 0.1
ina = INA219(SHUNT_OHMS)
ina.configure()

# Initialize NEOM8N
RX = 12

# Initialize DS3231
ds3231 = SDL_DS3231.SDL_DS3231(1, 0x68)
ds3231.write_now()

# Initialize MG90S
servoPIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN,GPIO.OUT)
p = GPIO.PWM(servoPIN, 50)
p.start(6)

# Initialize Buzzer
buzzPIN = 26
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzPIN,GPIO.OUT)
GPIO.output(buzzPIN,GPIO.LOW)

# Initialize motor
motorPIN = 19
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(motorPIN,GPIO.OUT)
MOTOR = 0
fla = 1
z = 0

# # Initialize camera
# camera = picamera.PiCamera()
# camera.resolution = (640, 480)
# camera.framerate = 30

def XBEE():
    global PORT, BAUD_RATE, device
    device = XBeeDevice(PORT, BAUD_RATE)
    device.open()

def spin_servo_parachute():
    global servoPIN, p
    p.ChangeDutyCycle(3.5)

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

def launch_wait():
    print('Launch wait started!')

    global st, ALTITUDE

    while ALTITUDE <= 10 and st == 0:
        pass
    st = 1

    print('Launch wait completed!')

def ascent():
    print('Ascent started!')

    global st, ALTITUDE

    temporal = ALTITUDE

    while ALTITUDE - temporal >= 0:
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

    global st, ALTITUDE

    while ALTITUDE > 400:
        pass

    spin_servo_parachute()

    while ALTITUDE > 300:
        pass

    st = 4

    print('Descent completed!')

def tp_release():
    print('TP release started!')

    global st, motorPIN

    temporal = time.time()

    while time.time() - temporal < 5:
        GPIO.output(motorPIN, GPIO.HIGH)
    GPIO.output(motorPIN, GPIO.LOW)

    while ALTITUDE > 1:
        pass

    st = 5

    print('TP release completed!')

def landed():
    print('Process finished!')
    GPIO.output(buzzPIN,GPIO.HIGH)
    sleep(300)
    GPIO.output(buzzPIN,GPIO.LOW)

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

if _name_ == "_main_":

    XBEE()

    _thread.start_new_thread(readGroundStation,())
    _thread.start_new_thread(toGSfromC,())
    _thread.start_new_thread(toGSfromTP,())
    
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