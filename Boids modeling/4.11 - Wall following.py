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
        
        
            
    def moveTo(self, pos, vel, factor, sphere):
                
        distance = self.calculateDistance(pos)
        
        requiredTime = 0
        
        if(norm(vel) != 0):
            requiredTime = math.ceil(norm(distance)/(factor*norm(vel)))
        
        t = 0
        while(t < requiredTime):
            t += 1
            for i in range(0, len(self.pos)):
                self.pos[i] -= distance[i]/requiredTime
            self.updateSphere(sphere)
            time.sleep(0.05)
                
    def generateSphere(self, rad):
        return sphere(pos = vector(self.pos[0], self.pos[1], self.pos[2]), radius = rad, color=vector(1, 0, 0))
    
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

def obtainOffsetPath(wall, rad, sides, myPos, offset) :
    return paths.circle(pos=myPos, np=sides, radius=rad+offset)
        
def defineNGonalWall(number, sides, rad, center):
    wallParts = []
    pathParts = []
    for n in range(0, number):
        wallParts.append(paths.circle(radius = rad, np = sides, pos=center + vec(2*rad*n, 0, 0)))
        pathParts.append(obtainOffsetPath(wallParts[n], rad, sides, center + vec(2*rad*n, 0, 0), 1))
        
    wall = []
    path = []
    for wp in range(0, len(wallParts)):
        wall += wallParts[wp]
        path += pathParts[wp]
        
    return [wall, path]
    
offset = 0.1
radius = 0.5

[wallPath, pathPath] = defineNGonalWall(1, 10, 2, vector(0, 0, 0))

wall = extrusion(path=wallPath, shape=shapes.rectangle(width=offset, height=8))
wall.opacity = 0.5

pathToFollow = extrusion(path=pathPath, shape=shapes.rectangle(width=offset, height=8))
pathToFollow.opacity = 0

wallFollower = Agent([10, 0, 10])
agentSphere = wallFollower.generateSphere(radius)

previous = [0, 0, 0]

waitNewWall = 10
w = 0

while(True):
    
    w += 1
    
    points = []
    for point in pathPath:
        points.append([point.x, point.y, point.z])
        
    closestWallPoints = findClosests(wallFollower, points)
    
    closest = closestWallPoints[1]
    
    if(norm(difference(closest, previous)) <= 0.01 and w>0):
        closest = closestWallPoints[2]
    
    previous = copy.deepcopy(wallFollower.pos)

    wallFollower.moveTo(closest, difference(closest, wallFollower.pos), 0.1, agentSphere)
    
    if(w >= waitNewWall):
        
        pathToFollow.visible = False
        wall.visible = False
        
        del pathToFollow
        del wall
        
        rad = randrange(1, 5)
        
        [wallPath, pathPath] = defineNGonalWall(1, randrange(3, 10), rad, vector(0, 0, 0))

        wall = extrusion(path=wallPath, shape=shapes.rectangle(width=offset, height=8))
        wall.opacity = 0.5
        
        pathToFollow = extrusion(path=pathPath, shape=shapes.rectangle(width=offset, height=8))
        pathToFollow.opacity = 0
        
        w = 0
        
        wallFollower = Agent([2*rad, 0, 2*rad])
        
        agentSphere.visible = False
        
        agentSphere = wallFollower.generateSphere(radius)
        
        previous = [2*rad, 0, 2*rad]
        