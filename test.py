import logging
from sys import getsizeof
# TODO: more detailed error logs
try:
    from serial import Serial
except:
    logging.error("couldn't import serial")

from struct import *
from math import pi
from threading import Thread
from time import sleep

import os


COMMAND_SETPIDPARAM = 0xA6
COMMAND_SETPOINT = 0xA7
COMMAND_GETSTATE = b'\xA8'
COMMAND_GETSTATE_RESPONSE = b'\xA9'
COMMAND_ENCODER_RESET = b'\xAA'
COMMAND_START_SAMPLING_SPEEDS = b'\xAB'
COMMAND_STOP_SAMPLING_SPEEDS = b'\xAC'
port='/dev/ttyACM0' 
baudrate=9600

serialPort = Serial(port, baudrate)

batteryStatus = 100
sampling = False
speed1 = []
speed2 = []
pulses1 = 0
pulses2 = 0
lastPulses1 = 0
lastPulses2 = 0
setpoint1 = 0.0
setpoint2 = 0.0
max_speed = 100

def set_speeds(speed_1, speed_2):
    # if abs(speed_1) > abs(max_speed):
    #     speed_1 = max_speed
    # if abs(speed_2) > abs(max_speed):
    #     speed_2 = max_speed
    setpoint1 = speed_1
    setpoint2 = speed_2
    package = pack("<Bff", COMMAND_SETPOINT, speed_2, speed_1)
    try:
        serialPort.write(package)
    except:
        print("error setting speeds")

def read_delta_encoders_count_state():
    try:
        serialPort.write(COMMAND_GETSTATE)
        header = serialPort.read()
        if header == COMMAND_GETSTATE_RESPONSE:
            print ("Get state")
            pulses1 = serialPort.read()
            pulses1 += serialPort.read()
            pulses1 += serialPort.read()
            pulses1 += serialPort.read()
            pulses1, = unpack("i", pulses1)
            print (pulses1)
            pulses2 = serialPort.read()
            pulses2 += serialPort.read()
            pulses2 += serialPort.read()
            pulses2 += serialPort.read()
            pulses2, = unpack("i", pulses2)
            print (pulses2)

            batteryCharge, = unpack('B', serialPort.read())

            lastPulses1 = pulses1
            lastPulses2 = pulses2
            pulses1 = pulses1
            pulses2 = pulses2
            batteryStatus = batteryCharge
            delta_pulses1 = pulses1 - lastPulses1
            delta_pulses2 = pulses2 - lastPulses2
            return delta_pulses2, delta_pulses1, batteryStatus, 0, 0
    except:
        logging.error("reading encoders")
        return 0,0,0,0,0
def set_constants(kc, ki, kd):
    package = pack("<Bfff", COMMAND_SETPIDPARAM, kc, ki, kd)
    try:
        serialPort.write(package)
    except:
        logging.error("setting constants")

def reset_encoders():
    try:
        serialPort.write(COMMAND_ENCODER_RESET)
    except:
        logging.error("reseting encoders")

def start_sampling_speeds():
    try:
        serialPort.write(COMMAND_START_SAMPLING_SPEEDS)
        sampling = True
    except:
        logging.error("start sampling")

def stop_sampling_speeds():
    try:
        serialPort.write(COMMAND_STOP_SAMPLING_SPEEDS)
        sampling = False
    except:
        logging.error("stop sampling")

# For debuggin propuses only
# TODO: This should run in other thread
def run_sampler():
    sleepLapse = 0.01
    while True:
        if sampling == True:
            try:
                while serialPort.inWaiting() < 4:
                    sleep(sleepLapse)
                speed1 = serialPort.read()
                speed1 += serialPort.read()
                speed1 += serialPort.read()
                speed1 += serialPort.read()
                speed1, = unpack("f", speed1)

                while serialPort.inWaiting() < 4:
                    sleep(sleepLapse)
                speed2 = serialPort.read()
                speed2 += serialPort.read()
                speed2 += serialPort.read()
                speed2 += serialPort.read()
                speed2, = unpack("f", speed2)

                speed1.append(speed1)
                speed2.append(speed2)
            except:
                logging.error("running sampler")
        sleep(sleepLapse)

speedA = 0.0
speedB = 0.0
print("5")
sleep(1)
print("4")
sleep(1)
print("3")
sleep(1)
print("2")
sleep(1)
print("1")
sleep(1)
print("Setting setpoints...")
set_speeds(6.28*2, 6.28*2)
while True:
    #print(f"{speedA} - {speedB}")
    line = ""
    while serialPort.in_waiting > 0:
        line = serialPort.readline()
        print(line)
    #sleep(0.5)

    #[dp2, dp1, bs, a1, a2] =read_delta_encoders_count_state()
    #speedA = speedA - 0.1
    #speedB = speedB - 0.1
    #sleep(0.5)
    #print(f"Pulsos A:{dp2}")
    #print(f"Pulsos B:{dp1}")
