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
SERVO_FIRE = "P9_14" #OX FEED servo
SERVO_DISCO = "P8_13" #disconnect switch servo
TRX_DEVICE = "/dev/ttyO1"
POWER_ON_ALT = 1414   #altitude in meters of power on
CHUTE_DEPLOY = 910  #altitude to deploy main chute at
MIN_ALT	     = 1500  #target minimum altitude before coming back down
ERROR_LOG = '/home/osu_rocketry_hybrid/avionics_error.log'
DATA_LOG = '/home/osu_rocketry_hybrid/avionics_data.log'
CMD_LOG = '/home/osu_rocketry_hybrid/commands_received.log'

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
PWM.start(SERVO_FIRE, 9, 60)
PWM.start(SERVO_DISCO, 12, 60)

#Sorry Curtis
#I cant think of a more elegant way to add this flag
dict = {'time': 0, 'agl': 0, 'temp': 0, 'a_x': 0, 'a_y': 0, 'a_z': 0, 'g_x': 0, 'g_y': 0, 'g_z': 0, 'gps_fix': 0, 'lat': 0, 'long': 0, 'arm_cut': 0, 'start_cut': 0, 'xbee_errors': 0, 'm_x': 0, 'm_y': 0, 'm_z': 0, 'new_dat_flag': 0}

def xbee_th():
  #xbee initialization
  xbee = serial.Serial('/dev/ttyO1', 19200);
  xbee.write('xbee started\n')
  
  #Open a log file to record commands received by the xbee (Future: time received as well?)
  com_log = open(CMD_LOG, 'a')
  com_log.write("Initilizing\n")
  
  rocket_started = 0
  rocket_abort = 0
  discon = 0
  pwmdstop = 1
  pwmfstop = 1

  while not rocket_started or not rocket_abort:
    print "in the command loop\n"
    #read a line from the xbee
    cmd = xbee.readline().rstrip()
    print cmd
    
    #debug if statement to tell me whether arming/disconnect has occurred
    if (cmd == "launch" and (rocket_abort == 1 or discon == 0 or pwmfstop == 1 or rocket_started == 1)):
      print "Launch cannot be performed: Disconnect, arm, or other event has not occurred\n"
      xbee.write("ERR_DISCONNECT = 0, OR ERR_ARM = 0\n")
    
    if (cmd == "launch" and rocket_abort == 0 and discon == 1 and rocket_started == 0 and pwmfstop == 0):
        rocket_started = 1
        #sec
        #activate the cutter/igniter ematch
        GPIO.output(CUTTER_PIN, GPIO.HIGH)
        sleep(1.0) 
        GPIO.output(CUTTER_PIN, GPIO.LOW)
        
        #Activate ball servo
        PWM.set_duty_cycle(SERVO_FIRE, 10)
        sleep(3)
        
        #stop pwm to fire servo and trigger pwmfstop flag
        PWM.stop(SERVO_FIRE)
        pwmfstop = 1
        xbee.write("Launch process complete: TO THE SKIES!!!\n")
        #TODO Adjust Servo Cycle to suit new servo
      
    if (cmd == "abort" and rocket_abort == 0):
      print "abort command received \n"
      com_log.write("abort command received\n")
      PWM.set_duty_cycle(SERVO_DISCO, 9)
      sleep(6) # TODO: Is this necessary
      
      #set the servo cycle
      PWM.set_duty_cycle(SERVO_FIRE, 7)
      sleep(3)
      
      #stop PWM signals so servos don't drain battery (TEST THIS)
      PWM.stop(SERVO_DISCO)
      PWM.stop(SERVO_FIRE)
      
      #set flag variables so that the pwm channels can be checked later
      pwmdstop = 1
      pwmfstop = 1
      
      #write to standard output and xbee (won't be visible in field) and command log
      xbee.write("Launch abort successful\n")
      rocket_abort = 1
      discon = 1
      
      print "Rocket abort finished\n"
      com_log.write("Abort command finished\n")
    
    if (cmd == "disconnect" and discon == 0):
      #record command received in std out and command log
      print "disconnect command received\n"
      com_log.write("disconnect command received\n")
      
      #cycle disconnect servo
      PWM.set_duty_cycle(SERVO_DISCO, 9)
      #TODO sleep here?
      sleep(6)
      
      #set disconnect variable to 1 so the command can't be repeated
      PWM.stop(SERVO_DISCO)
      pwmdstop = 1
      discon = 1
      #record disconnect complete in log
      print "disconnect servo actuated"
      com_log.write("disconnect command finished")
      
      
      
    if (cmd == "cycle" and rocket_started == 0 and rocket_abort == 0):
      #make a note in standard output and in command log of command received
      print "cycle command received\n"
      com_log.write("cycle command received\n")
      
      #twitch the servos back and forth a bit so that we know they are functioning
      PWM.set_duty_cycle(SERVO_DISCO, 8)
      sleep(0.5)
      PWM.set_duty_cycle(SERVO_FIRE, 9)
      sleep(0.5)
      PWM.set_duty_cycle(SERVO_DISCO, 7)
      sleep(0.5)
      PWM.set_duty_cycle(SERVO_FIRE, 12)
      
      #making note that cycle command complete in standard output and command log
      print "cycle command complete\n"
      com_log.write("servo cycling complete\n")
      
    if (cmd == "reset"):
      com_log.write("reset command received\n")
      print "Reset command received: resetting Servos and launch variables"
      
      #check to see if PWM has been closed so program doesn't crash, and restart those channels (battery saving?)
      if(pwmfstop == 1):
        PWM.start(SERVO_FIRE, 12, 60)
        pwmfstop = 0
      if(pwmdstop == 1):
        PWM.start(SERVO_DISCO, 7, 60)
        pwmdstop = 0
      
      #Set the servos to their closed state
      PWM.set_duty_cycle(SERVO_FIRE, 12)
      PWM.set_duty_cycle(SERVO_DISCO, 7)
      
      #reset controlling safety flag variables
      discon = 0
      rocket_started = 0
      rocket_abort = 0
      
      #Make a note in command log
      com_log.write("reset command complete\n")
        
  
  #rocket should be firing now or aborted
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
