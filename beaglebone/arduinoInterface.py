#!/usr/bin/env python3

import numpy as np
import time
import serial
import paho.mqtt.client as mqtt
from pynput import keyboard

commandString = ""
commanded = False
done = False
def on_press(key):
    global commandString, commanded, done
    try:
        if(key.char=='w'):
            commandString="+Z"
            #commanded=True
        elif(key.char=='s'):
            commandString="-Z"
            #commanded=True
        elif(key.char=='a'):
            commandString="+X"
            #commanded=True
        elif(key.char=='d'):
            commandString="-X"
            #commanded=True
        elif(key.char=='q'):
            commandString="+X+Z"
            #commanded=True
        elif(key.char=='e'):
            commandString="-X+Z"
            #commanded=True
        elif(key.char=='z'):
            commandString="+X-Z"
            #commanded=True
        elif(key.char=='c'):
            commandString="-X-Z"
            #commanded=True
        elif(key.char=='r'):
            commandString="-Y"
            #commanded=True
        elif(key.char=='f'):
            commandString="+Y"
            #commanded=True
        elif(key.char=='x'):
            done=True
    except:
        pass

mqttBroker = "192.168.2.1"

client = mqtt.Client("Arduino")
client.connect(mqttBroker, keepalive=300)
client.subscribe([("goto",0), ("robostep", 0)])

ser = serial.Serial('/dev/ttyUSB0')

oldx = 0
oldy = 0
oldz = -47

def on_message(client, userdata, message):
    global ser, oldx, oldy, oldz, commandString
    if(message.topic=='goto'):
        desired = message.payload
        desired = desired.split()
        r_x = float(desired[0])
        r_y = float(desired[1])
        r_z = float(desired[2])
        oldx = r_x
        oldy = r_y
        oldz = r_z
        cmdString = "GOTO " + str(r_x) + "," + str(r_y) + "," + str(r_z) + "\n"
        ser.write(bytes(cmdString.encode('ascii')))
        print("Did GOTO ", cmdString)
    elif(message.topic=='robostep'):
        print("received command")
        command = message.payload
        command = command.decode()
        if(commandString==""):
            print("Received Command ", command)
            commandString = command

client.on_message = on_message
client.loop_start()

listener = keyboard.Listener(on_press=on_press)
listener.start()

print("Start reading")
while(not done):
    if(not commanded and commandString!=""):
        print("Send Command")
        print(commandString)
        commandString+="\n"
        ser.write(bytes(commandString.encode('ascii')))
        commandString=""
        print("Done Send")
        commanded=True
    if(ser.in_waiting>0):
        print("Reading feedback")
        buffer = ser.read(ser.in_waiting)
        buffer = buffer.split(b'\n')
        robotpos = buffer[-2]
        print("Buffer length %d" %(len(buffer)-1))
        robotpos_split = robotpos.split()
        r_x = float(robotpos_split[0])
        r_y = float(robotpos_split[1])
        r_z = float(robotpos_split[2])
        client.publish("robotpos", robotpos)
        print(r_x, r_y, r_z)
        sendstr = buffer[-3].decode()
        sendstr += " " + str(oldx) + " " + str(oldy) + " " + str(oldz)
        sendstr += " " + str(r_x) + " " + str(r_y) + " " + str(r_z)
        oldx = r_x
        oldy = r_y
        oldz = r_z
        print(sendstr)
        client.publish("robocomm", sendstr)
        commanded=False
print("Done")
ser.close()