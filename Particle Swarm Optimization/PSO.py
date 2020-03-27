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
import numpy as np

class Agent:
    def __init__(self, pos, vel):
        self.pos = pos # dimensions
        self.vel = vel # velocities
        self.bestPos = self.pos
        
    def bestLocalPosition(self, agents, quantity):
        quantity = min(quantity, len(agents))
        closestAgents = self.sortByDistance(agents)
        closestAgents = closestAgents[0:quantity]
        
        bestPos = self.bestPos
        
        for agent in closestAgents:
            if(fitness(agent.bestPos) > fitness(bestPos)):
                bestPos = agent.bestPos
        
        return bestPos
        
    def sortingKey(self, agent):
        return self.distance(agent)
        
    def computeVelocity(self, omega, c1, c2, agents, locality):
        self.vel = omega*np.array(self.vel) + c1*random()*np.array(self.distances(self.bestPos)) + c2*random()*np.array(self.distances(self.bestLocalPosition(agents, locality)))

    def computePosition(self, omega, c1, c2, agents, locality):
        self.computeVelocity(omega, c1, c2, agents, locality)
        self.pos = np.array(self.pos) + np.array(self.vel)
        if(fitness(self.pos) > fitness(self.bestPos)):
            self.bestPos = self.pos
                            
    def distance(self, target):
        sqDist = 0
        for i in range(0, len(self.pos)):
            sqDist += (self.pos[i] - target.pos[i])**2
        
        return math.sqrt(sqDist)
    
    def distanceComponent(self, target, component):
        return target[component] - self.pos[component]
        
    def distances(self, target):
        dist = []
        for d in range(0, len(self.pos)):
            dist.append(self.distanceComponent(target, d))
            
        return dist

    def generateSphere(self):
        posVector = None
        velVector = None
        
        if(len(self.pos) == 3):
            posVector = vector(self.pos[0], self.pos[1], self.pos[2])
            velVector = vector(self.vel[0], self.vel[1], self.vel[2])
        elif(len(self.pos) == 2):
            posVector = vector(self.pos[0], self.pos[1], 0)
            velVector = vector(self.vel[0], self.vel[1], 0)

        return sphere(pos = posVector, vel = velVector, radius=0.5, color=vector(0.7, 0.85, 0.9))
    
    def updateSphere(self, sphere):     
        if(len(self.pos) == 3):
            sphere.pos = vector(self.pos[0], self.pos[1], self.pos[2])
            sphere.vel = vector(self.vel[0], self.vel[1], self.vel[2])
        elif(len(self.pos) == 2):
            sphere.pos = vector(self.pos[0], self.pos[1], 0)
            sphere.vel = vector(self.vel[0], self.vel[1], 0)
    
    def sortByDistance(self, agents):
        sortedArray = sorted(agents, key= lambda x:x.sortingKey(self))
        return sortedArray
    
    
def toRadius(volume):
    return math.pow(3*volume/(4*math.pi), 1/3)
    
def generateMultipleAgents(num):
    agents = []
    agentSpheres = []
    for t in range(0, num):
        newAgent = Agent([0.1+randrange(0, 50)-25, 0.1+randrange(0, 50)-25], [0, 0])
        agents.append(newAgent)
        agentSpheres.append(newAgent.generateSphere())
        
    return [agents, agentSpheres]
    
def fitness(position):
    return min(position[1], 10)


canvas(width=1000, height=600) 
canvas.resizable = True

samePosition = False

locality = 30
numberAgents = 200

# Create agents (boids)
[agents, agentSpheres] = generateMultipleAgents(numberAgents)
    
# Iteration
while(samePosition == False):
    
    for a in range(0, len(agents)):
        agents[a].computePosition(0.7, 0.1, 0.1, agents, locality)
        agents[a].updateSphere(agentSpheres[a])
#        print(agents[a].pos)
    
    time.sleep(0.01)
