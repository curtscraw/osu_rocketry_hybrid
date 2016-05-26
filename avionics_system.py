import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART
import Adafruit_BBIO.PWM as PWM
import BMP180
import LSM9DS0
import TGY6114MD
import datetime
import serial
from time import sleep
from sys import exit
import subprocess

from thread import start_new_thread

import logging

#general needed values
X_MAG_ARR_LEN = 120
CUTTER_PIN = "P9_12"
SERVO_FIRE = "P8_13" #combustion chamber servo
SERVO_DISCO = "P9_14" #disconnect switch servo
TRX_DEVICE = "/dev/ttyO1"
POWER_ON_ALT = 1414   #altitude in meters of power on
CHUTE_DEPLOY = 910  #altitude to deploy main chute at
MIN_ALT	     = 1500  #target minimum altitude before coming back down
ERROR_LOG = '/home/osu_rocketry_hybrid/avionics_error.log'
DATA_LOG = '/home/osu_rocketry_hybrid/avionics_data.log'

#TODO
#Determine servo duty range, map range to angles, and set servo to closed during setup

#log all warnings or above
logging.basicConfig(filename=ERROR_LOG,level=logging.WARNING,)

#setup the gps and transmitter uart ports
UART.setup("UART1")
UART.setup("UART2")

#initialize the cutter pin, it is triggered on a high signal
#could be used for ematch?
GPIO.setup(CUTTER_PIN, GPIO.OUT)
GPIO.output(CUTTER_PIN, GPIO.LOW)

#Initialize Servos and make sure they go to starting position
PWM.start(SERVO_FIRE, 12, 60)
#TODO Need to map second servo WITHOUT FRYING IT

#Sorry Curtis
#I cant think of a more elegant way to add this flag
dict = {'time': 0, 'agl': 0, 'temp': 0, 'a_x': 0, 'a_y': 0, 'a_z': 0, 'g_x': 0, 'g_y': 0, 'g_z': 0, 'gps_fix': 0, 'lat': 0, 'long': 0, 'arm_cut': 0, 'start_cut': 0, 'xbee_errors': 0, 'm_x': 0, 'm_y': 0, 'm_z': 0, 'new_dat_flag': 0}

def xbee_th():
  #xbee initialization
  xbee = serial.Serial('/dev/ttyO1', 19200);
  xbee.write('xbee started\n')
 
  rocket_started = 0
  rocket_abort = 0

  while not rocket_started or not rocket_abort:
    print "in the loop"
    #read a line from the xbee
    cmd = xbee.readline().rstrip()
    print cmd
    
    if (cmd == "launch"):
        PWM.set_duty_cycle(SERVO_DISCO, 7)
        sleep(4)
        rocket_started = 1
        sec
        #activate the cutter/igniter ematch
        GPIO.output(CUTTER_PIN, GPIO.HIGH)
        sleep(1.0) 
        GPIO.output(CUTTER_PIN, GPIO.LOW)
        
        #Activate ball servo
        PWM.set_duty_cycle(SERVO_FIRE, 10)
        #TODO Adjust Servo Cycle to suit new servo

    if (cmd == "abort"):
      #TODO how to abort properly: Servo open full
      PWM.set_duty_cycle(SERVO_DISCO, 50)
      sleep(10) # TODO: Is this necessary
      PWM.stop(SERVO_DISCO)
      PWM.stop(SERVO_FIRE)
      PWM.cleanup()
      xbee.write("Launch abort successful\n")
      rocket_abort = 1
  
  #rocket should be firing now or aborted, lets 
  while True:
    xbee.write(str(dict) + "\n")
    sleep(1)

  xbee.close()

def poll_th():
  #data polling thread is main thread
  #setup gyro and altimeter
  alt = BMP180.BMP180(POWER_ON_ALT)
  #motion = LSM9DS0.LSM9DS0()
  gyro = LSM9DS0.LSM9DS0_GYRO()
  accel = LSM9DS0.LSM9DS0_ACCEL()
  
  f_log = open(DATA_LOG, 'a')
  f_log.write("starting log\n")
  
  last_measure = POWER_ON_ALT
  
  #sensors are up, start the xbee thread
  start_new_thread(xbee_th, ())
  
  while True:
    try:
      dict['time'] = datetime.datetime.utcnow()
      dict['agl'] = alt.read_agl()
      dict['temp'] = alt.read_temperature()
      
      last_measure = dict['agl']
      
      (x, y, z) = accel.read_accel()
      dict['a_x'] = x
      dict['a_y'] = y
      dict['a_z'] = z
  
      (x, y, z) = gyro.read()
      dict['g_x'] = x
      dict['g_y'] = y
      dict['g_z'] = z

      (x, y, z) = accel.read_magnetometer()
      dict['m_x'] = x
      dict['m_y'] = y
      dict['m_z'] = z
      dict['new_dat_flag'] = 1
    
      f_log.write(str(dict) + "\n")

    except IOError as e:
      try:
        logging.exception('Got I2C exception on main handler' + str(dict['time']))
        
        alt = BMP180.BMP180(last_measure)
        accel = LSM9DS0.LSM9DS0_ACCEL()
        gyro = LSM9DS0.LSM9DS0_GYRO()
      except:
        logging.exception('Gotexception on recovery attempt')
  
    except KeyboardInterrupt:
      break
    except:
      logging.exception('Got an exception on main handler')


#start the whole system
poll_th()

logging.debug('script exited, not expected!')
