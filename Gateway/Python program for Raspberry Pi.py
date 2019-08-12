
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""
Material
for gateway
(1) B-L072Z-LRWAN1, STM32L0 Discovery kit LoRa, Sigfox
(2) RPi 3B+

for sensor node
(1) I-NUCLEO-LRWAN1 shield, USI® STM32™ Nucleo expansion board for LoRa™
(2) Self designed PCB with STM32F103 chip on board

Functions of this program,
(1) Receiving data from USB port through UART.
(2) Updating data to ThingsBoard (an online dashbaord).
    Live demo in ThingsBoard,
    ==> https://demo.thingsboard.io/login
(3) telepot, fast alarm alert.
(4) Local data logging.

NOTE
(1) Burn
    p.s. The program for I-NUCLEO-LRWAN1 shield & B-L072 discovery board were modified, based on "Ping Pong" example in Firmware I-CUBE-LRWAN.
    a) "My Gateway"     to STM B-L072 discovery board through uVision
    b) "My Shield"      to I-NUCLEO-LRWAN1 shield
    c) "My Sensor Node" to self designed PCB with STM32F103 chip on board
(2) "B-L072Z-LRWAN1"                RPi 3B+
        [USB micro]   ---USB--->   [USB port]
(3) RPi 3B+ receives strings from B-L072Z-LRWAN1 discovery board through UART.
    - Port = /dev/ttyACM0
    - Baud = 115200
(4) Getting Token,
    a) Thingsboard ACCESS_TOKEN, https://thingsboard.io/docs/samples/raspberry/temperature/
    b) Telepot TOKEN_ACCESS_HTTP_API, Once created a new bot, the HTTP API token is shown in Telegram
    c) Telepot Chat_id, can be obtained from "telepot_Echo_owner_id.py"

Program based on an example from
https://thingsboard.io/docs/samples/raspberry/temperature/
"""


############# import for UART <BEGIN> #############
import serial
import time,sys
from random import randint
############# import for UART <END> ###############


############# import for MQTT to Thingsboard <BEGIN> ############################################
# Reference
# https://thingsboard.io/docs/samples/raspberry/temperature/
import os
import paho.mqtt.client as mqtt
import json

# Using online "live demo server" (under ThingsBoard) instead of a local "ThingsBoard" server
THINGSBOARD_HOST = 'demo.thingsboard.io'
# Demo dashboard token
#ACCESS_TOKEN = 'DEMO_TOKEN'
ACCESS_TOKEN = 'GGxxxEYrnnnnnbC'

# Data capture and upload interval in seconds.
INTERVAL=1

sensor_data = {'temperature': 0, 'flame': 0, 'gas': 0, 'gasb': 0, 'humidity': 0, 'rssi': 0}

next_reading = time.time() 

client = mqtt.Client()

# Set access token
client.username_pw_set(ACCESS_TOKEN)

# Connect to ThingsBoard using default MQTT port and 60 seconds keepalive interval
client.connect(THINGSBOARD_HOST, 1883, 60)

client.loop_start()
############# import for MQTT to Thingsboard <END> ##############################################


############# import for Telepot <BEGIN> #############
import datetime
import RPi.GPIO as GPIO
import telepot
from telepot.loop import MessageLoop
############# import for Telepot <BEGIN> #############


############# Setup for UART <BEGIN> ###################################
# Define the UART serial port to mini UART port ttyS0
# RPi 3B+ ==> (1) ARM Primecell PL011 ttyAMA0 <--> embedded Bluetooth module
#             (2) CPU, BCM2837 mini UART ttyS0 <--> free to use
#             (3) USB hub, ttyACMn, n = no. <--> free to use
# old RPi ==> (1) ARM Primecell PL011 ttyAMA0 <--> free to use

### For STM B-L072 discovery board
SERIAL_PORT = "/dev/ttyACM0"
ser = serial.Serial(SERIAL_PORT, baudrate = 115200, timeout = 0.3)
############# Setup for UART <END> #####################################


############# Setup for Local Data Logging <BEGIN> ###################################
from time import strftime
# Logging data locally
def log_temp(temperature,flame,gas):
    temperature = int(temperature)
    flame = int(flame)
    gas = int(gas)
    with open("/home/pi/Desktop/IoT_data.csv", "a") as log:
        log.write("{0},{1},{2},{3},{4}\n".format(strftime("%Y-%m-%d"), strftime("%H:%M:%S"), str(temperature),str(flame),str(gas)))
############# Setup for Local Data Logging <END> #####################################

# Record start time when this programe is executed
starttime = time.strftime("%a %b %d %Y %H:%M:%S", time.localtime())

chat_id = 0
id = 99999
temperature = '99999'
flame = '99999'
gas = '89'
gas_b = '9999'
humidity = '11'
rssivalue = '99'
index = 111
index_l = 999
index_t = 9999
index_h = 99999
index_m = 999999
index_r = 9999999
counting = 0

############# Telegram bot <BEGIN> #######################################
# Reference
# https://telepot.readthedocs.io/en/latest/
def action(msg):
    chat_id = msg['chat']['id']
    text = msg['text']
    print ('Received: %s' % text)
    telegram_bot.sendMessage (chat_id, message)
# Using Telegram bot, BotFather, to create new bot.
# Once created a new bot, the HTTP API token is shown in Telegram
#telegram_bot = telepot.Bot('TOKEN_ACCESS_HTTP_API')
telegram_bot = telepot.Bot('7801xxx27:AAxxxxxxsUSXxxxxxCrgiwxxxxxAw_vo')
############# Telegram bot <END> #######################################


##################################        #   #   #       ##########################################
################################## Main programme <BEGIN> ##########################################
##################################        #   #   #       ##########################################
try:
    print("{}\r\nRPi 3B+ UART program is ready".format(starttime))
    print("LoRa module:")
    
    sensor_data['temperature'] = 23
    sensor_data['flame'] = 12
    sensor_data['gas'] = 52
    sensor_data['humidity'] = 72
    sensor_data['rssi'] = -34
    
    while True:
        time.sleep(1.5)
        
        ser.flushInput()          # clear the input uart buffer
        datalora = ser.readline() # read uart buffer untill "\r\n" occurs
        
        """
        # Self-defined format of data frame from B-L072Z-LRWAN1 Discovery board
        # ==> IDnQQQQqqqqFFFFTTTTHHHH
        #        n = sensor node ID number
        #        Q = 1st MQ2 gas sensor data
        #        q = 2nd MQ2 gas sensor data
        #        F = flame sensor module data
        #        T = DS18B20 temperature data
        #        H = HTS221 humdidty data on the I-NUCLEO-LRWAN1 shield
        """
        if datalora == b"":
            # If the variable is empty
            print("## NO DATA! PLEASE CHECK! ##")
        else:
            timestamp = time.strftime("%d/%m %H:%M:%S", time.localtime())
            datalora = datalora.strip()        # remove space characters
            datalora = datalora.strip(b'\x00') # remove null byte
            datalora = datalora.decode()       # change data type, <bytes> to <string>, because of the below string comparison
            #print(datalora)
            for x in range(len(datalora)):
                if datalora[x] == "I" and datalora[x+1] == "D":
                    index = x
                if datalora[x] == "L":
                    index_l = x
                if datalora[x] == "T":
                    index_t = x
                if datalora[x] == "H":
                    index_h = x
                if datalora[x] == "M":
                    index_m = x
                if datalora[x] == "N":
                    index_n = x
                if datalora[x] == "R":
                    index_r = x
            #print("ID:{} L:{} T:{} A:{} B:{}".format(index,index_l,index_t,index_h,index_m))  
            #id = datalora[index+2]
            flame = datalora[index_l+1:index_t-1]
            temperature = datalora[index_t+1:index_h-1]
            humidity = datalora[index_h+1:index_m-1]
            gas = datalora[index_m+2:index_n]
            gas_b = datalora[index_n+1:index_r]
            rssivalue = datalora[index_r+1:len(datalora)]
            #log_temp(temperature,flame,gas)  #save the data in .cvs format locally
            print("{ti} ID0{i} Temp={te:<4} flame={fl:<4} GasA={gaa:<4} GasB={gab:<4} humid={hu:<2} RSSI={r} Len{le}".format(
                                                                                    ti=timestamp,
                                                                                    i=1,
                                                                                    te=temperature,
                                                                                    fl=flame,
                                                                                    gaa=gas,
                                                                                    gab=gas_b,
                                                                                    hu=humidity,
                                                                                    r=rssivalue,
                                                                                    le=len(datalora),
                                                                                    ty=type(datalora)))
            
            ############# MQTT to Thingsboard <BEGIN> #######################################
            #print(u"Temperature: {:g}\u00b0C, flame: {:g}%".format(temperature, flame))
            try:
                gas = 1000 - int(gas)
                humidity = 110 - int(humidity)
            except ValueError:
                pass
            sensor_data['temperature'] = temperature
            sensor_data['flame'] = flame
            sensor_data['gas'] = gas
            sensor_data['gasb']=gas_b
            sensor_data['humidity'] = humidity
            sensor_data['rssi'] = rssivalue
            # Sending flame and temperature data to ThingsBoard
            client.publish('v1/devices/me/telemetry', json.dumps(sensor_data), 1)
            next_reading += INTERVAL
            sleep_time = next_reading-time.time()
            #print("Data pushed to Thingsboard Successfully!")
            if sleep_time > 0:
                time.sleep(sleep_time)
            ############# MQTT to Thingsboard <END> #########################################
            
            ############# Telegram bot <BEGIN> #######################################
            # Because the int(<str>) function cannot convert an empty string to integer,
            # it shows "invalid literal for int() with base 10" if <str> is empty
            message = "** WARNING! FIRE MAY OCCUR! **\n{}".format(timestamp)
            # Send message to telegram if flame occurs
            try:
                if int(flame) > 300:
                    message = ("** WARNING! FIRE MAY OCCUR! **\n"
                       "{ti}\n"
                       "Flame level = {fl}\r\n"
                       "Temperature = {temp} deg\r\n"
                       "Humidity = {hu}%\r\n"
                       "Gas A level = {gaa}\r\n"
                        "Gas B level = {gab}\r\n".format(ti=timestamp,fl=flame,temp=temperature,hu=humidity,gaa=gas,gab=gas_b))
                    telegram_bot.sendMessage (274406752, message)   #274406752 = Chat ID in Telegram
            except ValueError:
                pass
            ############# Telegram bot <END> #########################################
                
except KeyboardInterrupt:
    downtime = time.strftime("%d.%m.%Y %a %H:%M:%S", time.localtime())
    #ser.write("{} RPi DOWN\r\n".format(downtime))
    print("\r\n{} KeyboardInterrupt".format(downtime))
    


except:
    errortime = time.strftime("%d.%m.%Y %a %H:%M:%S", time.localtime())
    print("Error occurs @{}".format(errortime))
    
finally:
    ser.close()  # close serial port
    pass

############# MQTT to Thingsboard <BEGIN> #######################
client.loop_stop()
client.disconnect()
############# MQTT to Thingsboard <END> #########################