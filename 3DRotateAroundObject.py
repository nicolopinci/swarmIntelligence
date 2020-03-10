#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  8 20:32:42 2020

@author: nicolo
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
          
    def moveTo(self, target, vx, vy):     
        signs = [1, 1]
        if(self.x > target.x):
            signs[0] = -1
        
        if(self.y > target.y):
            signs[1] = -1
            
        self.vx = vx
        self.vy = vy
        
        self.changePosition(self.x + signs[0]*(self.vx), self.y + signs[1]*(self.vy))
                
        
    def __str__(self):
        return "Target(x, y) = (" + str(self.x) + ", " + str(self.y) + ")"
        
class Agent:
    def __init__(self, x, y, vx, vy):
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
        return math.tan((self.y - target.y)/(self.x - target.x))
        
    def accelerateX(self, error):
        self.ax = error
        
    def accelerateY(self, error):
        self.ay = error
        
    def moveCircle(self, circleCenter, radius, stepNr, precision):
        self.x = circleCenter[0] + radius*math.cos(stepNr*precision/radius)
        self.y = circleCenter[1] + radius*math.sin(stepNr*precision/radius)


    def closestAgent(self, targetList):
        minDist = self.distance(targetList[0])
        minTarget = targetList[0]
        
        for t in targetList:
            d = self.distance(t)
            if(d < minDist):
                minDist = d
                minTarget = t
                
        return minTarget
            
            
    def moveTo(self, target, vx, vy):     
        signs = [1, 1]
        if(self.x > target.x):
            signs[0] = -1
        
        if(self.y > target.y):
            signs[1] = -1
            
        self.changePosition(self.x + signs[0]*(self.vx + 0.5*self.ax), self.y + signs[1]*(self.vy + 0.5*self.ay))
                
    def __str__(self):
        return "Agent(x, y) = (" + str(self.x) + ", " + str(self.y) + ")"
    
def samePlace(x1, x2, y1, y2, prec):
    if(abs(x1-x2)<=prec and abs(y1-y2)<=prec):
        return True
    else:
        return False
    
def toRadius(volume):
    return math.pow(3*volume/(4*math.pi), 1/3)

def generateSphere(agent):
    return sphere(pos=vector(agent.x,agent.y,0), radius=0.5, color=vector(0.7, 0.85, 0.9))
    
def generateMultipleAgents(num):
    agents = []
    agentSpheres = []
    for t in range(0, num):
        newAgent = Agent(randrange(0, 50)-25, randrange(0, 50)-25, 0, 0)
        agents.append(newAgent)
        agentSpheres.append(generateSphere(newAgent))
        
    return [agents, agentSpheres]
    


canvas(width=1000, height=600) 

samePosition = False

# Create center of rotation (leader)
centerRotation = Target(0, 0, 0, 0)
crSphere = generateSphere(centerRotation)
crSphere.color = vector(1, 0, 0)

# Create agents (boids)
[agents, agentSpheres] = generateMultipleAgents(200)

# Create a target (the point towards the center of rotation should move)
target = Agent(10, 10, 0, 0)
targetSphere = generateSphere(target)
targetSphere.color = vector(0, 1, 0)

stepNr = 0
precision = 0.1

# Iteration
while(samePosition == False):
    stepNr += 1
    for t in range(0, len(agents)):
        cr = [centerRotation.x, centerRotation.y, 0]
        radius = agents[t].distance(centerRotation)
        agents[t].moveCircle(cr, radius, stepNr, precision)
        agentSpheres[t].pos = vector(agents[t].x, agents[t].y, 0)
        
    time.sleep(1/200)
    
    if(not samePlace(target.x, centerRotation.x, target.y, centerRotation.y, 0.01)):
        centerRotation.moveTo(target, 0.01, 0.01)
        crSphere.pos = vector(centerRotation.x, centerRotation.y, 0)
