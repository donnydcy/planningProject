# -*- coding: utf-8 -*-
"""
Created on Mon Dec  5 10:39:41 2016

Planning class

@author: chiyud
"""

import numpy as np
from scipy import signal
from heapq import heappush, heappop
import itertools
import math

class myPlan():

    def __init__(self, worldMap=[], costMap=[], poseX=0, poseY=0, dim = 100):
        self.worldMap = worldMap
        self.costMap = costMap
        self.visitMap = np.zeros([dim,dim])
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
        self.deploy_ = 0
        self.m_block = 2

        self.UGVPath = np.array([])
        self.UAVPath = np.array([])
        
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
        # print(np.array(path))

        return np.array(path)

        
        
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
        
# --------------------------------------------------------------
#                      Hacked by Anqi
# --------------------------------------------------------------
        
    # def generateUGVScoreMap(self, IG_map = [], m_block = 0.,Dmax = 0, deploy_ = False ):
    def generateUGVScoreMap(self, Dmax = 0):
        #TODO use euclidean? or other matric?
    
        # IG_map = np.array(IG_map)
        # score = np.divide(IG_map, self.distMap4UGV) 
        score = np.divide(self.IG_map, self.distMap4UGV)
        score = score*(1-self.occupMap)
        # if deploy_ :
            # m_block = 1/m_block
        
        # score *= m_block


        if self.deploy_:
            for i in range(dim):
                for j in range(dim):
                    if self.occupMap[j,i]:
                        continue
                    if self.distMap4UAV[j,i] <= Dmax:
                        score[j,i] *= 1
                    else:
                        score[j,i] *= Dmax/self.distMap4UAV[j,i] * self.penalty

        return score


# --------------------------------------------------------------
#                      Ends here
# --------------------------------------------------------------


# --------------------------------------------------------------
#                      Anqi's code begins here
# --------------------------------------------------------------


    def generateUAVScoreMap(self,Dmax = 0):

        score = np.divide(self.IG_map, self.distMap4UAV)
        
        for i in range(dim):
            for j in range(dim):
                if self.distMap4UGV[j,i] <= Dmax:
                    score[j,i] *= 1
                else:
                    score[j,i] *= Dmax/self.distMap4UGV[j,i] * self.penalty

        return score

    def UAVGoBackPlan(self, UGVPath, UAVLoc, batteryLevel):
        for i in range(UGVPath.shape[0]):
            if np.max(np.abs(UGVPath[i,:]-UAVLoc)) <= batteryLevel:
                r_step_ = i
                break

        r_loc_ = UGVPath[r_step_,:]

        path = []
        x_ = UAVLoc
        for i in range(r_step_):
            if r_loc_[0] > x_[0]:
                x_[0] = x_[0]+1
            if r_loc_[0] < x_[0]:
                x_[0] = x_[0]-1

            if r_loc_[1] > x_[1]:
                x_[1] = x_[1]+1
            if r_loc_[1] < x_[1]:
                x_[1] = x_[1]-1
            
            path.append(x_)

        path.reverse()
        path = np.array(path)

        UAVPath = UGVPath
        UAVPath[:r_step_,:] = path
        self.deploy_ = 0

        return UAVPath


    def updateVisitMap(self,UAVPath,UGVPath):
        if self.deploy_:
            self.visitMap[UAVPath[:,1],UAVPath[:,0]] += 1
            self.visitMap[UGVPath[:,1],UGVPath[:,0]] += 1
        else
            self.visitMap[UGVPath[:,1],UGVPath[:,0]] += 1


    def updateIGMap(self):
        IG_Map_ = np.divide(((self.m_block-1)*self.worldMap+1),self.visitMap)
        self.IG_Map = signal.convolve2d(IG_Map_,np.ones((3,3)),mode='same')  


    def recordPath(self,UGVPath_,UAVPath_):
        path_len_ = min(UGVPath_.shape[0],UAVPath_.shape[0])
        self.UGVPath = np.append(self.UGVPath, UGVPath_[:path_len_,:], axis=0)
        self.UAVPath = np.append(self.UAVPath, UAVPath_[:path_len_,:], axis=0)
        assert self.UGVPath.shape[0] == self.UAVPath.shape[0]


    def nearBlock(self,UGVX,UGVY):
        for i in range(-1,2):
            for j in range(-1,2)
                if self.occupMap[UGVX+i,UGVY+j]:
                    return True

        return False
# --------------------------------------------------------------
#                      Anqi's code ends here
# --------------------------------------------------------------
        
        
        
        
        
        
        


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

# --------------------------------------------------------------
#                      Anqi's code begins here
# --------------------------------------------------------------



# --------------------------------------------------------------
#                      Anqi's code ends here
# --------------------------------------------------------------


a.runAstar(0,0,20,20)