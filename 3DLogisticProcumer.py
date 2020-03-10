#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 14:57:17 2020

@author: nicolo
"""

from vpython import *
import time
import math

class Resource:
    def __init__(self, amount):
        self.amount = amount
        
    def applyLogistic(self, a, b):
        self.amount = max(a*self.amount - b*self.amount*self.amount, 0)
        
    def __str__(self):
        return "The amount of available resources is " + str(self.amount)
        
class Agent:
    def __init__(self):
        self.value = 0
        
    def produceValue(self, resource):
        consumed = resource.amount
        self.value = consumed
        
    def __str__(self):
        return "The value produced by the agent is " + str(self.value) 
        
def toRadius(volume):
    return math.pow(3*volume/(4*math.pi), 1/3)

sourceSphere = sphere(pos=vector(-1,0,0), radius=0.5, color=vector(1,0.7,0.2))
connection = rod = cylinder(pos=vector(-1,0,0), axis=vector(2,0,0), radius=0.05)
targetSphere = sphere(pos=vector(1,0,0), radius=0.5)    

totalTime = int(input("Insert the total amount of time to simulate: "))
print("The resource generator is described by the formula L(t+1) = a*L(t) - b*L(t)*L(t).")
a = float(input("Please choose a: "))
b = float(input("Please choose b: "))

agent = Agent()
resource = Resource(1)

totalValue = 0

t = 0

while(t < totalTime):
    resource.applyLogistic(a, b)
    agent.produceValue(resource)
    
    totalValue += agent.value
    
    connection.axis = vector(1.5 + toRadius(totalValue),0,0)
#    sourceSphere.radius = resource.amount
    
    targetSphere.radius = toRadius(totalValue)
    time.sleep(0.2)
    t += 1
    
