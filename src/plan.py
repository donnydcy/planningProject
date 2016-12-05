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
        self.distMap4UGV = np.zeros([dim,dim])
        self.distMap4UAV = np.zeros([dim,dim])
        self.dim = dim
        self.PQ = priorityQ()
        self.AstarPQUGV = priorityQ()
        self.AstarPQUAV = priorityQ()
        
        self.AstarWeight = 1.05
        self.generateDistMap()
        
        self.penalty = 0.8
        
    def generateDistMap(self, isUAV = False):
        # run Dijkstra
        A_closelist = {}
        parent= {}
        self.PQ.add_task(self.pix2ind([self.UGVY, self.UGVX]),0,0)
        while(len(self.PQ.pq)>0):
            priority,g, key = self.PQ.pop_task()
            
           
            A_closelist[key] = g 
            
            pos  = self.ind2pix(key)
            y = pos[0]
            x = pos[1]
            if isUAV:
                self.distMap4UAV[y,x] = g
            else:
                self.distMap4UGV[y,x] = g
            
            # move left:    
            self.move(parent, A_closelist, x-1, y, key, g, isUAV_flag = isUAV)
            # move up-left:    
            self.move(parent, A_closelist, x-1, y-1, key, g, isUAV_flag = isUAV)
            # move up:    
            self.move(parent, A_closelist, x, y-1, key, g, isUAV_flag = isUAV)
            # move up-right:    
            self.move(parent, A_closelist, x+1, y-1, key, g, isUAV_flag = isUAV)
            # move right:
            self.move(parent, A_closelist, x+1, y, key, g, isUAV_flag = isUAV)
            # move down-right:
            self.move(parent, A_closelist, x+1, y+1, key, g, isUAV_flag = isUAV)
            # move down:
            self.move(parent, A_closelist, x, y+1, key, g, isUAV_flag = isUAV)
            # move down -left
            self.move(parent, A_closelist, x-1, y+1, key, g, isUAV_flag = isUAV)
                    
                
        pass
        
      
    def runAstar(self, startX, startY, goalX, goalY, isUAV = False):
          
        A_closelist = {}
        parent= {}
        self.PQ.add_task(self.pix2ind([startY, startX]),0,0)
        while(len(self.PQ.pq)>0):
            priority,g, key = self.PQ.pop_task()
   
            A_closelist[key] = g 

            # if find goal, break the loop
            if key == self.pix2ind([goalY, goalX]):
                  break

            pos  = self.ind2pix(key)
            y = pos[0]
            x = pos[1]

            # move left:    
            self.move(parent, A_closelist, x-1, y, key, g, A_flag = True, isUAV_flag = isUAV)
            # move up-left:    
            self.move(parent, A_closelist, x-1, y-1, key, g, A_flag = True, isUAV_flag = isUAV)
            # move up:    
            self.move(parent, A_closelist, x, y-1, key, g, A_flag = True, isUAV_flag = isUAV)
            # move up-right:    
            self.move(parent, A_closelist, x+1, y-1, key, g, A_flag = True, isUAV_flag = isUAV)
            # move right:
            self.move(parent, A_closelist, x+1, y, key, g, A_flag = True, isUAV_flag = isUAV)
            # move down-right:
            self.move(parent, A_closelist, x+1, y+1, key, g, A_flag = True, isUAV_flag = isUAV)
            # move down:
            self.move(parent, A_closelist, x, y+1, key, g, A_flag = True, isUAV_flag = isUAV)
            # move down -left
            self.move(parent, A_closelist, x-1, y+1, key, g, A_flag = True, isUAV_flag = isUAV)
            

        # get path from Astar result
        node = self.pix2ind([goalY, goalX])
        path = []        
        path.append([goalY,goalX])
        while node in parent:
            new_node = parent[node]
            path.append(self.ind2pix(new_node))
            node = new_node

        path.reverse()
        print(np.array(path))


        
        
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
    def move(self, parents, closeList,X,Y, preTask, pre_g, A_flag = False, isUAV_flag = False):
        new_pose = self.checkBoarder([Y,X])
        cost = 1
        if new_pose :
            if not A_flag:
                heristic = 0
            else:
                # if use A*, herisitic gerenated from distMap
                if isUAV_flag:                    
                    heristic = self.distMap4UAV[Y,X] * self.AstarWeight
                    cost = 1
                else:
                    heristic = self.distMap4UGV[Y,X] * self.AstarWeight                    
                    cost = self.costMap[Y,X]
                    
            new_pose_ind = self.pix2ind(new_pose)
            if new_pose_ind not in closeList:
                if self.PQ.add_task(new_pose_ind, pre_g, cost, h = heristic):
                    parents[new_pose_ind] = preTask
        
        
    def generateUGVScoreMap(self, IG_map = [], m_block = 0.,Dmax = 0, deploy_ = False ):
        #TODO use euclidean? or other matric?
    
        IG_map = np.array(IG_map)
        score = np.divide(IG_map, self.distMap4UGV) 
        if deploy_ :
            m_block = 1/m_block
        
        score *= m_block
        
        
        if not deploy_:
            for i in range(dim):
                for j in range(dim):
                    if self.distMap4UGV[j,i] <= Dmax:
                        score[j,i] *= 1
                    else:
                        score[j,i] *= Dmax/self.distMap4UGV[j,i] * self.penalty
        
        
        
        
        
        
        
        
        
        
        


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


cost = np.ones([100,100])
a = myPlan(worldMap=np.zeros([100,100]), costMap=cost, poseX=0, poseY=0, dim = 100)
a.runAstar(0,0,20,20)