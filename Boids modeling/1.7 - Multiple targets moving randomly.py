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
    def __init__(self, x, y, vx, vy, value):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.value = value
        
    def changeVelocity(self, vx, vy):
        self.vx = vx
        self.vy = vy

    def changePosition(self, x, y):
        self.x = x
        self.y = y
          
    def randomMove(self):
        self.x = max(min(self.x + randrange(0, 2) - 1, 10), -10)
        self.y = max(min(self.y + randrange(0, 2) - 1, 10), -10)

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
        return math.tan((self.y - target.y)/(self.x - target.x))
        
    def accelerateX(self, error):
        self.ax = error
        
    def accelerateY(self, error):
        self.ay = error
        
    def closestTarget(self, targetList):
        minDist = self.distance(targetList[0])
        minTarget = targetList[0]
        
        for t in targetList:
            d = self.distance(t)
            if(d < minDist):
                minDist = d
                minTarget = t
                
        return minTarget
            
    def maxValueTarget(self, targetList):
        maxValue = targetList[0].value
        maxTarget = targetList[0]
        
        for t in targetList:
            if(t.value > maxValue):
                maxValue = t.value
                maxTarget = t
                
        return maxTarget
    
        
    def moveTo(self, target):     
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
    
def generateMultipleTargets(num):
    targets = []
    targetSpheres = []
    for t in range(0, num):
        newTarget = Target(randrange(0, 10)-5, randrange(0, 10)-5, 0, 0, randrange(0, 50))
        targets.append(newTarget)
        targetSpheres.append(generateSphere(newTarget))
        targetSpheres[t].color = vector(0, newTarget.value/50, 0)
        
    return [targets, targetSpheres]
    
def lookForSpheres(target, targetSpheres):
    
    spheres = []
    
    for t in targetSpheres:
        if(target.x == t.pos.x and target.y == t.pos.y):
            spheres.append(t)
    
    return spheres
    
samePosition = False
agent = Agent(0, 0, 0, 0, 1)
[targets, targetSpheres] = generateMultipleTargets(10)

currentClosest = agent.maxValueTarget(targets)

distanceX = agent.distanceX(currentClosest)
distanceY = agent.distanceY(currentClosest)

agentSphere = generateSphere(agent)
agentSphere.color = vector(1, 0, 0)

totalValue = 0
collectedValue = 0

toBeDeleted = None

for t in targets:
    totalValue += t.value
    
while(samePosition == False):
    agent.moveTo(currentClosest)
    agentSphere.pos = vector(agent.x, agent.y, 0)

    indexDelete = None
    
    for t in range(0, len(targets)):
        if(not targets[t] == toBeDeleted):
            targets[t].randomMove()
            targetSpheres[t].pos = vector(targets[t].x, targets[t].y, 0)
        else:
            indexDelete = t
            
    if(not toBeDeleted==None and not indexDelete==None):
        del targets[indexDelete]
        targetSpheres[indexDelete].visible = False
        del targetSpheres[indexDelete]
        
    currentClosest = agent.maxValueTarget(targets)
    
    
    currentDistanceX = agent.distanceX(currentClosest)
    currentDistanceY = agent.distanceY(currentClosest)

    agent.accelerateX(distanceX)
    agent.accelerateY(distanceY)

    distanceX = currentDistanceX
    distanceY = currentDistanceY
        
    time.sleep(1)
    
    if(samePlace(agent.x, currentClosest.x, agent.y, currentClosest.y, 0.1)):
        collectedValue += currentClosest.value
        toBeDeleted = currentClosest
        
    if(collectedValue == totalValue):
        samePosition = True
        
    print("Performance: " + str(collectedValue/totalValue))