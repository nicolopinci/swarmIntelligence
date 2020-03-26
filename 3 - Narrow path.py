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
#        agents.remove(self)
        sortedArray = sorted(agents, key= lambda x:x.sortingKey(self))
#        agents.append(self)
        return sortedArray
    
    def separate(self, agents, minAllowed, locality):
        sortedAgents = self.sortByDistance(agents)
        numberDimensions = len(self.dim)
        randomDirection = [0, 0, 0]
        for d in range(0, numberDimensions):
            if(d == 2):
                randomDirection[d] = random()*(-1)
            else:
                randomDirection[d] = random() - 0.5

            
        k = 0
        while(self.minDistance(sortedAgents[0:locality]) < minAllowed):
            k += 1
            newPosition = []
            for d in range(0, numberDimensions):
                newPosition.append(self.dim[d] + k*randomDirection[d])
            newAgPos = Agent(newPosition, [0, 0, 0], [0, 0, 0], '')
            self.changeAcceleration(self.distances(newAgPos), 10)
            self.moveTo(newAgPos)
            
    def cohere(self, agents, cohesion):
        correctedCohesion = max(0, min(cohesion, len(agents)))
        sortedAgents = self.sortByDistance(agents)
        currentCenter = findCurrentCenter(sortedAgents[0:correctedCohesion])
        centerAgent = Agent(currentCenter, [0, 0, 0], [0, 0, 0], 'fictitiousCenter')
        
        for l in range(0, len(currentCenter)):
            self.acc[l] = 0.2*self.distanceC(centerAgent, l)
                 
        self.moveTo(centerAgent)
            
            
            
def toRadius(volume):
    return math.pow(3*volume/(4*math.pi), 1/3)
    
def generateMultipleAgents(num):
    agents = []
    agentSpheres = []
    for t in range(0, num):
        newAgent = Agent([0.1+randrange(0, 50)-25, 0.1+randrange(0, 50)-25, 0.1+randrange(5, 50)], [0, 0, 0], [0, 0, 0], 'agent')
        agents.append(newAgent)
        agentSpheres.append(newAgent.generateSphere())
        
    return [agents, agentSpheres]
    
    
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

def calculateDeltaSqPosition(pos1, pos2):
    deltaPos = calculateDelta(pos1, pos2)
    dp = 0
    for d in range(0, len(deltaPos)):
        dp += deltaPos[d]*deltaPos[d]
        
    return dp
        
def generateLastPositions(agents):
    out = []
    for agent in agents:
        out.append(0)
        
    return out
    
canvas(width=1000, height=600) 
canvas.resizable = True

samePosition = False

separation = 5
cohesion = 10
numberAgents = 100

# Prepare narrow path
leftWall = box(pos=vector(-10, 0, 0), length=19, height=20, width=2, color=vector(0.776, 0.886, 0.890), opacity=0.8)
rightWall = box(pos=vector(10, 0, 0), length=19, height=20, width=2, color=vector(0.776, 0.886, 0.890), opacity=0.8)

# Leader
leader = Agent([0, 0, 10], [0, 0, 0], [0, 0, 0], 'leader')
leaderSphere = leader.generateSphere()
leaderSphere.color = vector(1, 0, 0)

# Create agents (boids)
[agents, agentSpheres] = generateMultipleAgents(numberAgents)

# Create a target
target = Agent([0, 0, -100], [0, 0, 0], [0, 0, 0], 'target')
targetSphere = target.generateSphere()
targetSphere.color = vector(0, 1, 0)

leaderQueue = []
leaderQueue.append(copy.deepcopy(leader).dim)
lastPositions = generateLastPositions(agents)


# Iteration
while(samePosition == False):
           
    closestAgents = leader.sortByDistance(agents) # finds the closest agents to the leader
    agIndex = 0
    previousAgent = None
        
    for a in range(0, len(closestAgents)): # consider the agents in order
        for t in range(0, len(agents)): # finds the index of the closest agent               
            if(closestAgents[a].samePlace(agents[t], 0.0001)):
                agIndex = t        
                        
        posToMoveIdx = lastPositions[agIndex] + 1

        if(a < len(leaderQueue)-1 and posToMoveIdx < len(leaderQueue)):
            previous = Agent(leaderQueue[posToMoveIdx], [0, 0, 0], [0, 0, 0], 'positional')
            
            remainHere = False
            
            for otherAgent in agents:
                if(otherAgent.distance(previous) < 1):
                    remainHere = True
                    
            if(remainHere == False):
                agents[agIndex].changeAcceleration(agents[agIndex].distances(previous), 0.2)
                agents[agIndex].moveTo(previous)
    
            for p in range(0, len(leaderQueue)):
                if(agents[agIndex].samePlace(Agent(leaderQueue[p], [0, 0, 0], [0, 0, 0], ''), 1)):
                    lastPositions[agIndex] = p
                    
    
#    for agent in agents:
#        agent.separate(agents, separation, 5)
#        agent.cohere(agents, cohesion)
        
        
    for t in range(0, len(agents)):
        agents[t].updateSphere(agentSpheres[t])

    
    if(not target.samePlace(leader, 0.1)):
        leader.changeAcceleration(leader.distances(target), 0.01)
        leader.moveTo(target)
        newPosition = copy.deepcopy(leader.dim)
        leader.updateSphere(leaderSphere)
        if(calculateDeltaSqPosition(newPosition, leaderQueue[-1]) >= 2):
            leaderQueue.append(copy.deepcopy(leader).dim)
            
    time.sleep(0.05)