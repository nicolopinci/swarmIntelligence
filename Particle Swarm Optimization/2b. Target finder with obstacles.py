#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  8 20:32:42 2020

@author: nicolo
"""
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("TkAgg")
import math
from random import randrange
import random
from vpython import *
import time
import copy
import numpy as np

class Obstacle:
    def __init__(self, pos, side):
        self.pos = pos
        self.side = side
        
    def pointIntersection(self, agent):
        if(agent.distance(self.pos) <= 0.5 + self.side*1.414):
            return True
        else:
            return False
        
    def lineIntersection(self, pos1, pos2):
        foundIntersection = False
        
        t = 0
        distance12 = Agent(pos1, np.zeros(len(pos1))).distance(pos2)
        deltaT = None
        
        if(distance12 > 1):
            deltaT = 1/distance12
        else:
            deltaT = 1
        
        while(foundIntersection == False and t<=1):
            currentPosition = np.array(pos1) + t*np.array(pos2)
            if(self.pointIntersection(Agent(currentPosition, np.zeros(len(currentPosition)))) == True):
                foundIntersection = True
            t += deltaT
        
        return foundIntersection
    
    def represent(self):
        posVector = None
        
        if(len(self.pos) == 3):
            posVector = vector(self.pos[0], self.pos[1], self.pos[2])
        elif(len(self.pos) == 2):
            posVector = vector(self.pos[0], self.pos[1], 0)

        obs = box(pos=posVector, length=self.side, height=self.side, width=self.side, color=vector(0, 0, 1)) 
        return obs

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
            if(target.distance(agent.bestPos) < target.distance(bestPos)):
                bestPos = agent.bestPos
        
        return bestPos
        
    def sortingKey(self, agent):
        return self.distance(agent.pos)
        
    def computeVelocity(self, omega, c1, c2, agents, locality):
        self.vel = omega*np.array(self.vel) + c1*random()*np.array(self.distances(self.bestPos)) + c2*random()*np.array(self.distances(self.bestLocalPosition(agents, locality)))

    def computePosition(self, w0, wf, k, maxK, c1, c2, agents, locality, target):
        omega = defineOmega(w0, wf, k, maxK)
        self.computeVelocity(omega, c1, c2, agents, locality)
        self.pos = np.array(self.pos) + np.array(self.vel)
        if(target.distance(self.pos) < target.distance(self.bestPos)):
            self.bestPos = self.pos
                            
    def distance(self, targetPos):
        sqDist = 0
        for i in range(0, len(self.pos)):
            sqDist += (self.pos[i] - targetPos[i])**2
        
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
        
        return sphere(pos = posVector, vel = velVector, radius=0.5, color=vector(0.7, 0.85, 0.9), opacity = 0.1)
    
    def updateSphere(self, sphere):     
        if(len(self.pos) == 3):
            sphere.pos = vector(self.pos[0], self.pos[1], self.pos[2])
            sphere.vel = vector(self.vel[0], self.vel[1], self.vel[2])
        elif(len(self.pos) == 2):
            sphere.pos = vector(self.pos[0], self.pos[1], 0)
            sphere.vel = vector(self.vel[0], self.vel[1], 0)
        
        sphere.opacity = 0.1
        
    def sortByDistance(self, agents):
        sortedArray = sorted(agents, key= lambda x:x.sortingKey(self))
        return sortedArray
    
    
def toRadius(volume):
    return math.pow(3*volume/(4*math.pi), 1/3)
    
def generateMultipleOrigins(num, originPosition, maxVel):
    agents = []
    agentSpheres = []
    for t in range(0, num):
        newAgent = Agent(originPosition, np.zeros(len(originPosition)))
        for vc in range(0, len(originPosition)):
            newAgent.vel[vc] = random()*maxVel - maxVel/2
            
        agents.append(newAgent)
        agentSpheres.append(newAgent.generateSphere())
        
    return [agents, agentSpheres]
    
def defineTarget():
    targetAgent = Agent([0, 0], [0, 0])
    targetAgent.generateSphere().color = vec(1, 0, 0)
    return targetAgent
    

def defineOmega(w0, wfin, currentIteration, numberIterations):
    return w0 - (w0 - wfin)*currentIteration/numberIterations

def drawBestSphere(bestPosition):
    posVector = None
    
    if(len(bestPosition) == 3):
        posVector = vector(bestPosition[0], bestPosition[1], bestPosition[2])
    elif(len(bestPosition) == 2):
        posVector = vector(bestPosition[0], bestPosition[1], 0)
        
    return sphere(pos = posVector, radius=0.5, color=vector(1, 1, 0))

canvas(width=1000, height=600, title='Particle Swarm Optimization') 
canvas.resizable = True
            
# Definition of parameters given as input
numberAgents = int(input("Number of agents: "))
maxK = int(input("Number of iterations: "))
w0 = float(input("Initial weight (w0): ")) 
wf = float(input("Final weight (wf): "))
c2 = float(input("Local search factor: "))
c1 = float(input("Global search factor: "))
locality = int(input("Number of agents in a cluster: "))
maxVel = float(input("Maximum velocity: "))

# Create agents (boids)
[agents, agentSpheres] = generateMultipleOrigins(numberAgents, [10, 10], maxVel)
target = defineTarget()

currentMinimum = target.distance(agents[0].pos)
minPos = agents[0].pos

# Source for drawing with Spyder: https://stackoverflow.com/questions/10944621/dynamically-updating-plot-in-matplotlib
plt.ion()
class DynamicUpdate():
    def on_launch(self, symbol):
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[], symbol)
        self.ax.set_autoscaley_on(True)
        self.ax.grid()

    def on_running(self, xdata, ydata):
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        self.ax.relim()
        self.ax.autoscale_view()
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

fitnessGraph = DynamicUpdate()
pathGraph = DynamicUpdate()

xdata = []
ydata = []
    
fitnessGraph.on_launch('o')
pathGraph.on_launch('o')

pathGraph.on_running(agents[0].pos[0], agents[0].pos[1])
pathGraph.on_running(target.pos[0], target.pos[1])


k = 0 

bestPath = []
obstacles = []

# Obstacle definition
for o in range(0, 40):
    xRand = randrange(0, 2)
    yRand = randrange(0, 2)
    
    xMult = None
    yMult = None
    
    if(xRand == 0):
        xMult = -1
    else:
        xMult = 1
        
    if(yRand == 0):
        yMult = -1
    else:
        yMult = 1
        
    obstacles.append(Obstacle([randrange(1, 8)*xMult, randrange(1, 8)*yMult], 1))
    obstacles[o].represent()
    
# Iteration
while(k < maxK):
    
    k += 1
    
    for a in range(0, len(agents)):
        previousAgent = copy.deepcopy(agents[a])
        agents[a].computePosition(w0, wf, k, maxK, c1, c2, agents, locality, target)
        agents[a].updateSphere(agentSpheres[a])
        
        if(target.distance(agents[a].pos) <= currentMinimum):
            collision = False
            for ob in obstacles:
                if(ob.lineIntersection(agents[a].pos, previousAgent.pos)):
                    collision = True
            if(collision == False):
                currentMinimum = target.distance(agents[a].pos)
                minPosition = agents[a].pos
      
    xdata.append(k)
    ydata.append(currentMinimum)
    bestPath.append(copy.deepcopy(minPosition))
    fitnessGraph.on_running(xdata, ydata)
    
    pathGraph.on_running([i[0] for i in bestPath], [i[1] for i in bestPath])
    
    agents = []
    
    for agentSphere in agentSpheres:
        agentSphere.visible = False
        del agentSphere
            
    [agents, agentSpheres] = generateMultipleOrigins(numberAgents, minPosition, maxVel)

    for a in range(0, len(agents)):
        agents[a].updateSphere(agentSpheres[a])
        
    bestSphere = drawBestSphere(minPosition)
    
    time.sleep(0.1)