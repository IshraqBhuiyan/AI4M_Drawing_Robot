#!/usr/bin/env python3
import pygame
import time
import paho.mqtt.client as mqtt

mqttBroker = "192.168.2.1"
client = mqtt.Client("touchscreen")
client.connect(mqttBroker, keepalive=300)

BLUE = (0,0,255)
WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (255,0,0)
GREEN = (0,255,0)

screen = None

def on_message(client, userdata, message):
    global screen
    if(message.topic=='clear'):
        screen.fill(BLACK)
        pygame.display.flip()
    elif(message.topic=='drawline'):
        payload = message.payload
        payload = payload.split()
        l_x = float(payload[0])
        l_y=  float(payload[1])
        l_x2 = float(payload[2])
        l_y2 = float(payload[3])
        pygame.draw.line(screen, GREEN,(l_x, l_y), (l_x2, l_y2), width=15)
        pygame.display.flip()
    elif(message.topic=='save'):
        payload = message.payload
        fname = payload.decode()
        print("Saving as ", fname)
        pygame.image.save(screen, fname)


client.loop_start()
client.subscribe([("clear",0), ("drawline",0), ("save", 0)])
client.on_message = on_message

pygame.init()
infoObject = pygame.display.Info()

screen = pygame.display.set_mode((infoObject.current_w, infoObject.current_h))
#screen = pygame.display.set_mode((800,480))

def drawCircle(screen, x, y):
    pygame.draw.circle(screen, BLUE, (x,y), 10)
isPressed=True

#pygame.event.set_blocked(None)
#pygame.event.set_allowed(pygame.MOUSEMOTION)
#pygame.event.set_allowed(pygame.MOUSEBUTTONDOWN)
#pygame.event.set_allowed(pygame.MOUSEBUTTONUP)
clock = pygame.time.Clock()
oldx = 0
oldy=0
while True:
    #pygame.event.pump()
    #(x,y) = pygame.mouse.get_pos()
    #drawCircle(screen,x,y)
    #if(x!=oldx and y!=oldy):
    #    print(x,y)
    #    oldx=x
    #    oldy=y
    #clock.tick(20)
    for event in pygame.event.get():
        if event.type == pygame.FINGERDOWN or event.type==pygame.FINGERMOTION:
            x = int(event.x)
            y = int(event.y)
            if(event.type==pygame.FINGERDOWN):
                print("fdown")
                publishString = str(event.x) + " " + str(event.y)
                client.publish("fingerdown", publishString)
            if(x!=oldx and y!=oldy):
                print(x,y, round(event.dx, 2), round(event.dy, 2))
                oldx = x
                oldy = y
                drawCircle(screen, x, y)
        elif event.type== pygame.FINGERUP:
            print("fup")
       # if event.type == pygame.MOUSEBUTTONDOWN:
       #     isPressed = True
       # elif event.type == pygame.MOUSEBUTTONUP:
       #     isPressed = True
       # elif event.type == pygame.MOUSEMOTION and isPressed:
       #     (x,y) = pygame.mouse.get_pos()
       #     drawCircle(screen, x,y)
    clock.tick(30)
    pygame.display.flip()
