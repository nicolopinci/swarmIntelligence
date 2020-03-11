#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  8 20:32:42 2020

@author: nicolo
"""

import math
from random import randrange
import random
from vpython import *
import time

class Agent:
    def __init__(self, dim, vel, acc, agentType):
        self.dim = dim # dimensions
        self.vel = vel # velocities
        self.acc = acc # accelerations
        self.agentType = agentType
        
    def changeAcceleration(self, accelerations, factor):
        self.acc = accelerations
        for a in self.acc:
            a = a*factor
        
    def changeVelocity(self, velocities):
        self.vel = velocities

    def changePosition(self, pos):
        self.dim = pos
          
    def moveTo(self, target): 
        for i in range(0, len(self.dim)):
            sign = 1
            if(self.dim[i] > target.dim[i]):
                sign = -1
                
            self.dim[i] += sign*(self.vel[i] + 0.5*self.acc[i])
                            
    def distance(self, target):
        sqDist = 0
        for i in range(0, len(self.dim)):
            sqDist += (self.dim[i] - target.dim[i])**2
        
        return math.sqrt(sqDist)
    
    def distanceC(self, target, component):
        return abs(self.dim[component] - target.dim[component])
        
    def distances(self, target):
        dist = []
        for d in range(0, len(self.dim)):
            dist.append(self.distanceC(target, d))
            
        return dist

    def closestAgent(self, targetList):
        minDist = self.distance(targetList[0])
        minTarget = targetList[0]
        
        for t in targetList:
            d = self.distance(t)
            if(d < minDist):
                minDist = d
                minTarget = t
                
        return minTarget
                
    def samePlace(self, target, prec):
        for i in range(0, len(self.dim)):
            if(abs(self.dim[i] - target.dim[i]) > prec):
                return False
        
        return True
    
    def generateSphere(self):
        return sphere(pos=vector(self.dim[0], self.dim[1], self.dim[2]), radius=0.5, color=vector(0.7, 0.85, 0.9))
    
    def updateSphere(self, sphere):
        sphere.pos = vector(self.dim[0], self.dim[1], self.dim[2])
    
    def minDistance(self, agents):
        minDistance = float('inf')
        for a in agents:
            if(self.distance(a) < minDistance and a != self):
                minDistance = self.distance(a)
                
        return minDistance
        
def toRadius(volume):
    return math.pow(3*volume/(4*math.pi), 1/3)
    
def generateMultipleAgents(num):
    agents = []
    agentSpheres = []
    for t in range(0, num):
        newAgent = Agent([0.1+randrange(0, 50)-25, 0.1+randrange(0, 50)-25, 0.1+randrange(0, 50)-25], [0, 0, 0], [0, 0, 0], 'agent')
        agents.append(newAgent)
        agentSpheres.append(newAgent.generateSphere())
        
    return [agents, agentSpheres]
    
def separate(agents, minAllowed):
    for a in agents:
        numberDimensions = len(a.dim)
        randomDirection = [0, 0, 0]
        for d in range(0, numberDimensions):
            randomDirection[d] = random() - 0.5
            
        k = 0
        while(a.minDistance(agents) < minAllowed):
            k += 1
            a.dim = [a.dim[0] + k*randomDirection[0], a.dim[1] + k*randomDirection[1], a.dim[2] + k*randomDirection[2]]
     
        
def findCurrentCenter(agents):
    numDim = len(agents[0].dim)
    center = []
    
    for n in range(0, numDim):
        componentCenter = 0
        for agent in agents:
            componentCenter += agent.dim[n]
        center.append(componentCenter/len(agents))
        
    return center

def calculateDelta(v1, v2):
    v3 = []
    for i in range(0, len(v1)):
        v3.append(v1[i] - v2[i])
        
    return v3
    
canvas(width=1000, height=600) 

samePosition = False

separation = 4
cohesion = 5

# Create center of rotation (leader)
centerRotation = Agent([0, 0, 0], [0, 0, 0], [0, 0, 0], 'leader')
crSphere = centerRotation.generateSphere()
crSphere.color = vector(1, 0, 0)

# Create agents (boids)
[agents, agentSpheres] = generateMultipleAgents(50)

# Create a target (the point towards the center of rotation should move)
target = Agent([10, 10, 0], [0, 0, 0], [0, 0, 0], 'target')
targetSphere = target.generateSphere()
targetSphere.color = vector(0, 1, 0)

# Iteration
while(samePosition == False):
    
    currentCenter = findCurrentCenter(agents)
    delta = calculateDelta(currentCenter, centerRotation.dim)
    
    for t in range(0, len(agents)):
        cr = centerRotation.dim
        radius = agents[t].distance(centerRotation)
        agents[t].changeAcceleration(agents[t].distances(centerRotation), 0.000000001)
        
        newPosition = []
        
        for i in range(0, len(centerRotation.dim)):
            newPosition.append(centerRotation.dim[i] + delta[i])
            
        newAgent = Agent(newPosition, [0, 0, 0], [0, 0, 0], 'tempTarget')
        
        agents[t].moveTo(newAgent)
            
    separate(agents, separation)
    
    for t in range(0, len(agents)):
        agents[t].updateSphere(agentSpheres[t])
    
    if(not target.samePlace(centerRotation, 1)):
        centerRotation.changeAcceleration(centerRotation.distances(target), 0.1)
        centerRotation.moveTo(target)
        centerRotation.updateSphere(crSphere)
        
    else:
        target.dim[0] = randrange(-20, 20)
        target.dim[1] = randrange(-20, 20)
        target.dim[2] = randrange(-20, 20)
        target.updateSphere(targetSphere)
        
    time.sleep(1/100)
