# -*- coding: utf-8 -*-
"""
Created on Mon Dec  5 10:39:41 2016

Planning class

@author: chiyud
"""

import numpy as np
from heapq import heappush, heappop
import itertools
import math

class myPlan():
    def __init__(self, worldMap=[], costMap=[], poseX=0, poseY=0, dim = 100):
        self.worldMap = worldMap
        self.costMap = costMap
        self.UGVX = poseX
        self.UGVY = poseY
        self.UAVX = poseX
        self.UAVY = poseY
        self.distMap = np.zeros([dim,dim])
        self.dim = dim
        self.PQ = priorityQ()
            
        self.generateDistMap()
        
    def generateDistMap(self):
        # run Dijkstra
        A_closelist = {}
        parent= {}
        self.PQ.add_task(self.pix2ind([self.UGVY, self.UGVX]),0,0);
        while(len(self.PQ.pq)>0):
            priority,g, key = self.PQ.pop_task()
            
           
            A_closelist[key] = g 
            
            pos  = self.ind2pix(key)
            y = pos[0]
            x = pos[1]
            self.distMap[y,x] = g            
            
            # move left:    
            self.move(parent, A_closelist, x-1, y, key, g)
            
            # move up-left:    
            self.move(parent, A_closelist, x-1, y-1, key, g)
            # move up:    
            self.move(parent, A_closelist, x, y-1, key, g)
            # move up-right:    
            self.move(parent, A_closelist, x+1, y-1, key, g)
            # move right:
            self.move(parent, A_closelist, x+1, y, key, g)
            # move down-right:
            self.move(parent, A_closelist, x+1, y+1, key, g)
            # move down:
            self.move(parent, A_closelist, x, y+1, key, g)
            # move down -left
            self.move(parent, A_closelist, x-1, y+1, key, g)
                    
                
            pass
        
    def pix2ind(self,pix = [0,0]):
        return pix[0]*self.dim + pix[1]
        
    def ind2pix(self, ind =0):
        return [math.floor(ind/self.dim), math.floor(math.fmod(ind, self.dim))]
        
    def checkBoarder(self, pix = [0, 0]):
        if pix[0] >=0 and pix[0]<self.dim and pix[1] >=0 and pix[1] < self.dim:
            return pix
        else:
            False
                
    # move a cell and insert to pq, a building block to Dijkstra
    def move(self, parents, closeList,X,Y, preTask, pre_g):
        new_pose = self.checkBoarder([Y,X])
        if new_pose :
            new_pose_ind = self.pix2ind(new_pose)
            if new_pose_ind not in closeList:
                if self.PQ.add_task(new_pose_ind, pre_g, self.costMap[Y,X]):
                    parents[new_pose_ind] = preTask
        
        
        


class priorityQ:# This class is modified from Python.org
    def __init__(self):
        
        self.pq = []                         # list of entries arranged in a heap
        self.entry_finder = {}               # mapping of tasks to entries
        self.REMOVED = '<removed-task>'      # placeholder for a removed task
        self.counter = itertools.count()     # unique sequence count

    def add_task(self, task, pre_g=0, c = 0, h = 0):
    #'Add a new task or update the priority of an existing task'
        priority = pre_g + c + h
        g = pre_g + c
        if task in self.entry_finder:
            if self.entry_finder[task][1] > g:
                self.remove_task(task)
            else:
                return False
            
        count = next(self.counter)
        entry = [priority, g, count, task]
        self.entry_finder[task] = entry
        heappush(self.pq, entry)
        return task

    def remove_task(self,task):
    #'Mark an existing task as REMOVED.  Raise KeyError if not found.'
        entry = self.entry_finder.pop(task)
        entry[-1] = self.REMOVED

    def pop_task(self):
        #'Remove and return the lowest priority task. Raise KeyError if empty.'
        while self.pq:
            priority, g, count, task = heappop(self.pq)
            if task is not self.REMOVED:
                del self.entry_finder[task]
                return priority, g, task
                raise KeyError('pop from an empty priority queue')

a =    myPlan(worldMap=np.zeros([100,100]), costMap=np.ones([100,100]), poseX=0, poseY=0, dim = 100)
