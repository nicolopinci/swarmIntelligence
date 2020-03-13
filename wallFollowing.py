#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 12 22:25:45 2020

@author: nicolo
"""

from random import randrange
from vpython import *
import math
import time
import copy
import numpy as np

class Agent:
    def __init__(self, pos):
        self.pos = pos
        
    def calculateDistance(self, pos):
        delta = []
        
        for i in range(0, len(self.pos)):
            delta.append(self.pos[i] - pos[i])
            
        return delta
        
        
            
    def moveTo(self, pos, vel):
                
        distance = self.calculateDistance(pos)
        
        requiredTime = 0
        
        if(norm(vel) != 0):
            requiredTime = math.ceil(norm(distance)/norm(vel))
        
        t = 0
        while(t < requiredTime):
            t += 1
            for i in range(0, len(self.pos)):
                self.pos[i] -= distance[i]/requiredTime
                
    def generateSphere(self, rad):
        return sphere(pos = vector(self.pos[0], self.pos[1], self.pos[2]), radius = rad)
    
    def updateSphere(self, sphere):
        sphere.pos = vector(self.pos[0], self.pos[1], self.pos[2])

def norm(v):
    delta = 0
    for i in range(0, len(v)):
        delta += (v[i])**2
        
    return math.sqrt(delta)
    
def findClosests(agent, wallPoints):
    return sorted(wallPoints, key=lambda x:norm(agent.calculateDistance(Agent(x).pos)))
    
def difference(v1, v2):
    v3 = []
    for i in range(0, len(v1)):
        v3.append(v2[i] - v1[i])
        
    return v3
    
def addOffset(pos, offset, acivate):
    for i in range(0, len(pos)):
        if(activate[i] == True):
            pos[i] += offset
    
offset = 0.2
radius = 0.5

wallPath = paths.circle(radius = 2)
#wallPath = paths.rectangle(width=100, height=8)
wall = extrusion(path=wallPath, shape=shapes.rectangle(width=offset, height=8))

wallFollower = Agent([10, 0, 10])
agentSphere = wallFollower.generateSphere(radius)

previous = [0, 0, 0]

while(True):
    points = []
    for point in wallPath:
        points.append([point.x, point.y, point.z])
        
    closestWallPoints = findClosests(wallFollower, points)
    
    closest = closestWallPoints[1]
    
    if(norm(difference(closest, previous)) <= 0.1):
        closest = closestWallPoints[2]
    
    previous = copy.deepcopy(wallFollower.pos)

    wallFollower.moveTo(closest, difference(closest, wallFollower.pos))
    wallFollower.updateSphere(agentSphere)
        
    time.sleep(0.1)