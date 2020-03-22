#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
The agent moves towards a fixed target maintaining its velocity constant as it approaches the target.
"""

import math
from random import randrange
from vpython import *
import time

class Target:
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        
    def changeVelocity(self, vx, vy):
        self.vx = vx
        self.vy = vy

    def changePosition(self, x, y):
        self.x = x
        self.y = y
          
    def randomMove(self):
        self.x = self.x + randrange(0, 10)-5
        self.y = self.y + randrange(0, 10)-5

    def __str__(self):
        return "Target(x, y) = (" + str(self.x) + ", " + str(self.y) + ")"
        
class Agent:
    def __init__(self, x, y, vx, vy, mass):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.ax = 0
        self.ay = 0
        
    def changeVelocity(self, vx, vy):
        self.vx = vx
        self.vy = vy

    def changePosition(self, x, y):
        self.x = x
        self.y = y
        
    def distance(self, target):
        return math.sqrt((self.x - target.x)**2 + (self.y - target.y)**2)
    
    def distanceX(self, target):
        return abs(self.x - target.x)
    
    def distanceY(self, target):
        return abs(self.y - target.y)
    
    def angle(self, target):
        return math.atan((self.y - target.y)/(self.x - target.x))
        
    def accelerateX(self, error):
        self.ax = error/10
        
    def accelerateY(self, error):
        self.ay = error/10
        
    def moveTo(self, target):     
        self.changePosition(self.x + self.vx + 0.5*self.ax, self.y + self.vy + 0.5*self.ay)
                
    def __str__(self):
        return "Agent(x, y) = (" + str(self.x) + ", " + str(self.y) + ")"
    
def samePlace(x1, x2, y1, y2, prec):
    if(abs(x1-x2)<=prec and abs(y1-y2)<=prec):
        return True
    else:
        return False
    
def toRadius(volume):
    return math.pow(3*volume/(4*math.pi), 1/3)

agentSphere = sphere(pos=vector(0,0,0), radius=0.5, color=vector(1,0.7,0.2))
targetSphere = sphere(pos=vector(10,10,0), radius=0.5)    

samePosition = False
agent = Agent(0, 0, 0, 0, 1)
target = Target(10, 10, 10, 10)

distanceX = agent.distanceX(target)
distanceY = agent.distanceY(target)
initialDistance = agent.distance(target)

performance = 0
previousPerformance = 0

velocityFactor = randrange(2, 40)

agent.changeVelocity(distanceX/velocityFactor, distanceY/velocityFactor) # I assume it is possible to control the velocity of the agent, and not only the force

while(samePosition == False):
    agent.moveTo(target)
    
    performance = max(0, 1-agent.distance(target)/initialDistance)

    if(performance >= previousPerformance):
        agentSphere.pos = vector(agent.x, agent.y, 0)
        targetSphere.pos = vector(target.x, target.y, 0)
        print("Performance: " + str(performance)) 
        previousPerformance = performance
    else:
        samePosition = True
            
    time.sleep(0.1)
