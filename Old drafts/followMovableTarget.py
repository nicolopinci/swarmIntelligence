#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  8 20:32:42 2020

@author: nicolo
"""

import math
from random import randrange

class Target:
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        
    def changeVelocity(self, vx, vy):
        self.vx = vx
        self.vy = vy

    def changePosition(self, x, y):
        self.x = x
        self.y = y
          
    def randomMove(self):
        self.x = self.x + randrange(0, 10)-5
        self.y = self.y + randrange(0, 10)-5

    def __str__(self):
        return "Target(x, y) = (" + str(self.x) + ", " + str(self.y) + ")"
        
class Agent:
    def __init__(self, x, y, vx, vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        
    def changeVelocity(self, vx, vy):
        self.vx = vx
        self.vy = vy

    def changePosition(self, x, y):
        self.x = x
        self.y = y
        
    def distance(self, target):
        return math.sqrt((self.x - target.x)**2 + (self.y - target.y)**2)
    
    def angle(self, target):
        return math.atan((self.y - target.y)/(self.x - target.x))
        
    def moveTo(self, target):
      
        dx = target.x - self.x
        dy = target.y - self.y
        
        vx = dx/5
        vy = dy/5
        
        self.changeVelocity(vx, vy)
        self.changePosition(self.x + vx, self.y + vy)
                
    def __str__(self):
        return "Agent(x, y) = (" + str(self.x) + ", " + str(self.y) + ")"
    
def samePlace(x1, x2, y1, y2, prec):
    if(abs(x1-x2)<=prec and abs(y1-y2)<=prec):
        return True
    else:
        return False
    
samePosition = False
agent = Agent(0, 0, 0, 0)
target = Target(10, 10, 10, 10)

while(samePosition == False):
    target.randomMove()
    agent.moveTo(target)
    print(agent)
    print(target)
    print("..........")
    
    if(samePlace(agent.x, target.x, agent.y, target.y, 0.000001)):
        samePosition = True
