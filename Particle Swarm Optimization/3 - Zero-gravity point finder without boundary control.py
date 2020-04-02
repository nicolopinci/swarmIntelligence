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

class CelestialBody:
    def __init__(self, pos, mass):
        self.pos = pos
        self.mass = mass
        
    def generateSphere(self):
        posVector = None
        velVector = None
        
        if(len(self.pos) == 3):
            posVector = vector(self.pos[0], self.pos[1], self.pos[2])
        elif(len(self.pos) == 2):
            posVector = vector(self.pos[0], self.pos[1], 0)

        return sphere(pos = posVector, radius=toRadius(self.mass), color=vector(0, 1, 1))
        
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
            if(fitness(agent.bestPos, celestialBodies) > fitness(bestPos, celestialBodies)):
                bestPos = agent.bestPos
        
        return bestPos
        
    def sortingKey(self, agent):
        return self.distance(agent)
        
    def computeVelocity(self, omega, c1, c2, agents, locality):
        self.vel = omega*np.array(self.vel) + c1*random()*np.array(self.distances(self.bestPos)) + c2*random()*np.array(self.distances(self.bestLocalPosition(agents, locality)))

    def computePosition(self, w0, wf, k, maxK, c1, c2, agents, locality):
        omega = defineOmega(w0, wf, k, maxK)
        self.computeVelocity(omega, c1, c2, agents, locality)
        self.pos = np.array(self.pos) + np.array(self.vel)
        if(fitness(self.pos, celestialBodies) > fitness(self.bestPos, celestialBodies)):
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
        newAgent = Agent([0.1+randrange(0, 50)-25, 0.1+randrange(0, 50)-25, 0.1+randrange(0, 50)-25], [0, 0, 0])
        agents.append(newAgent)
        agentSpheres.append(newAgent.generateSphere())
        
    return [agents, agentSpheres]
    
def generateMultipleBodies(num):
    celestialBodies = []
    celestialSpheres = []
    for t in range(0, num):
        newBody = CelestialBody([0.1+randrange(0, 50)-25, 0.1+randrange(0, 50)-25, 0.1+randrange(0, 50)-25], randrange(1, 1000))
        celestialBodies.append(newBody)
        celestialSpheres.append(newBody.generateSphere())
        
    return [celestialBodies, celestialSpheres]
    
def fitness(position, celestialBodies):   
    
    totalForce = np.array([0.0, 0.0, 0.0])
    agent = Agent(position, [0, 0, 0])
    
    for cel in celestialBodies:
        totalForce = totalForce + np.array(agent.distances(cel.pos))*cel.mass*abs(math.pow(agent.distance(cel), -3))
        
    return np.linalg.norm(totalForce)*(-1)


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
numberCelestial = int(input("Number of celestial bodies: "))
# Create agents (boids)
[agents, agentSpheres] = generateMultipleAgents(numberAgents)
[celestialBodies, celestialSpheres] = generateMultipleBodies(numberCelestial)

currentMaximum = fitness(agents[0].pos, celestialBodies)

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

d = DynamicUpdate()

xdata = []
ydata = []
    
d.on_launch()

k = 0 
  
# Iteration
while(k < maxK):
    
    k += 1
    
    for a in range(0, len(agents)):
        agents[a].computePosition(w0, wf, k, maxK, c1, c2, agents, locality)
        agents[a].updateSphere(agentSpheres[a])
        
        if(fitness(agents[a].pos, celestialBodies) >= currentMaximum):
            currentMaximum = fitness(agents[a].pos, celestialBodies)
            
    xdata.append(k)
    ydata.append(currentMaximum)
    d.on_running(xdata, ydata)
        
    time.sleep(0.001)