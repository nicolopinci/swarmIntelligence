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
    
def generateMultipleOrigins(num, originPosition):
    agents = []
    agentSpheres = []
    for t in range(0, num):
        newAgent = Agent(originPosition, np.zeros(len(originPosition)))
        for vc in range(0, len(originPosition)):
            newAgent.vel[vc] = random()*2-1
            
        agents.append(newAgent)
        agentSpheres.append(newAgent.generateSphere())
        
    return [agents, agentSpheres]
    
def defineTarget():
    targetAgent = Agent([0, 0], [0, 0])
    targetAgent.generateSphere().color = vec(1, 0, 0)
    return targetAgent
    

def defineOmega(w0, wfin, currentIteration, numberIterations):
    return w0 - (w0 - wfin)*currentIteration/numberIterations

canvas(width=1000, height=600, title='Particle Swarm Optimization') 
canvas.resizable = True

# Create agents in the goal positions (for debug purposes)
# In this example the minimum is a plane, defined as x = 0, therefore I generate a se of agents with x = 0 and variable y and z
'''for a in range(-20, 20):
    for b in range(-20, 20):
        maximumAgent = Agent([0, a, b], [0, 0, 0])
        maxSphere = maximumAgent.generateSphere()
        maxSphere.color = vec(1, 0, 0)
        maxSphere.opacity = 0.3'''
            
# Definition of parameters given as input
numberAgents = int(input("Number of agents: "))
maxK = int(input("Number of iterations: "))
w0 = float(input("Initial weight (w0): ")) 
wf = float(input("Final weight (wf): "))
c2 = float(input("Local search factor: "))
c1 = float(input("Global search factor: "))
locality = int(input("Number of agents in a cluster: "))

# Create agents (boids)
[agents, agentSpheres] = generateMultipleOrigins(numberAgents, [10, 10])
target = defineTarget()

currentMinimum = target.distance(agents[0].pos)
minPos = agents[0].pos

# Source for drawing with Spyder: https://stackoverflow.com/questions/10944621/dynamically-updating-plot-in-matplotlib
plt.ion()
class DynamicUpdate():
    def on_launch(self):
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[], 'o')
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
    
fitnessGraph.on_launch()
pathGraph.on_launch()

pathGraph.on_running(agents[0].pos[0], agents[0].pos[1])
pathGraph.on_running(target.pos[0], target.pos[1])


k = 0 

bestPath = []
  
# Iteration
while(k < maxK):
    
    k += 1
    
    for a in range(0, len(agents)):
        agents[a].computePosition(w0, wf, k, maxK, c1, c2, agents, locality, target)
        agents[a].updateSphere(agentSpheres[a])
        
        if(target.distance(agents[a].pos) <= currentMinimum):
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
            
    [agents, agentSpheres] = generateMultipleOrigins(numberAgents, minPosition)

    for a in range(0, len(agents)):
        agents[a].updateSphere(agentSpheres[a])
        
    time.sleep(0.1)