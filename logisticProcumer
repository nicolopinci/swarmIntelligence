#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  8 19:24:08 2020

@author: nicolo
"""

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
        consumed = min(resource.amount, 10)
        self.value = consumed
        resource.amount = resource.amount - consumed
        
    def __str__(self):
        return "The value produced by the agent is " + str(self.value) 
        
totalTime = int(input("Insert the total amount of time to simulate: "))
print("The resource generator is described by the formula L(t+1) = a*L(t) - b*L(t)*L(t).")
a = int(input("Please choose a: "))
b = int(input("Please choose b: "))

agent = Agent()
resource = Resource(1)

totalValue = 0

t = 0

while(t < totalTime):
    resource.applyLogistic(a, b)
    agent.produceValue(resource)
    totalValue += agent.value
    
    print(resource)
    print(agent)
    print("The total value produced so far is " + str(totalValue))
    print("----")
    
    t += 1