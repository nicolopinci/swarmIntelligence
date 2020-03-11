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
import copy

class Agent:
    def __init__(self, dim, vel, acc, agentType):
        self.dim = dim # dimensions
        self.vel = vel # velocities
        self.acc = acc # accelerations
        self.agentType = agentType
        
    def sortingKey(self, agent):
        return self.distance(agent)
        
    def changeAcceleration(self, accelerations, factor):
        self.acc = accelerations
        for a in range(0, len(self.acc)):
            self.acc[a] = self.acc[a]*factor
        
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
    
    def sortByDistance(self, agents):
        agents.remove(self)
        sortedArray = sorted(agents, key= lambda x:x.sortingKey(self))
        agents.append(self)
        return sortedArray
        
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
    
def separate(agents, minAllowed, locality):

    for a in agents:
        sortedAgents = a.sortByDistance(agents)
        numberDimensions = len(a.dim)
        randomDirection = [0, 0, 0]
        for d in range(0, numberDimensions):
            randomDirection[d] = random() - 0.5
            
        k = 0
        while(a.minDistance(sortedAgents[0:locality]) < minAllowed):
            k += 1
            newPosition = []
            for d in range(0, numberDimensions):
                newPosition.append(a.dim[d] + k*randomDirection[d])
            a.moveTo(Agent(newPosition, [0, 0, 0], [0, 0, 0], ''))
     
        
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

def cohere(agents, cohesion):
    correctedCohesion = max(0, min(cohesion, len(agents)))
    for agent in agents:
        sortedAgents = agent.sortByDistance(agents)
        currentCenter = findCurrentCenter(sortedAgents[0:correctedCohesion])
        centerAgent = Agent(currentCenter, [0, 0, 0], [0, 0, 0], 'fictitiousCenter')
        
        for l in range(0, len(currentCenter)):
            agent.acc[l] = 0.2*agent.distanceC(centerAgent, l)
                 
        agent.moveTo(centerAgent)
        
    
canvas(width=1000, height=600) 
canvas.resizable = True

samePosition = False

separation = 5
cohesion = 3
numberAgents = 50

# Create center of rotation (leader)
centerRotation = Agent([0, 0, 0], [0, 0, 0], [0, 0, 0], 'leader')
crSphere = centerRotation.generateSphere()
crSphere.color = vector(1, 0, 0)

# Create agents (boids)
[agents, agentSpheres] = generateMultipleAgents(numberAgents)

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
        agents[t].changeAcceleration(centerRotation.acc, 1)
        
        newPosition = []
        
        for i in range(0, len(centerRotation.dim)):
            newPosition.append(agents[t].dim[i] - delta[i])
            
        newAgent = Agent(newPosition, [0, 0, 0], [0, 0, 0], 'tempTarget')

        agents[t].moveTo(newAgent)
            
    separate(agents, separation, 5)
    cohere(agents, cohesion)
    
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
        
    time.sleep(0.01)
