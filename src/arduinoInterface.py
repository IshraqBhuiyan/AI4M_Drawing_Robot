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
        print(key.char)
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
            print("get done")
            done=True
    except:
        pass

mqttBroker = "192.168.2.1"

client = mqtt.Client("Arduino")
client.connect(mqttBroker, keepalive=300)

ser = serial.Serial('/dev/cu.wchusbserial1410')

listener = keyboard.Listener(on_press=on_press)
listener.start()

print("Start reading")
while(not done):
    if(not commanded and commandString!=""):
        print("Send Command")
        print(commandString)
        commandString+="\n"
        ser.write(bytes(commandString.encode('ascii')))
        print("Done Send")
        commandString=""
        commanded=True
    if(ser.in_waiting>0):
        print("Reading feedback")
        buffer = ser.read(ser.in_waiting)
        buffer = buffer.split(b'\n')
        robotpos = buffer[-2]
        print("Buffer length %d" %(len(buffer)-1))
        robotpos = robotpos.split()
        r_x = float(robotpos[0])
        r_y = float(robotpos[1])
        r_z = float(robotpos[2])
        print(r_x, r_y, r_z)
        print(buffer[-3])
        commanded=False