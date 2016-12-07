# -*- coding: utf-8 -*-
"""
Created on Mon Dec  5 10:39:41 2016

Planning class

@author: chiyud
"""

import numpy as np
import scipy as sp
from scipy import signal
from heapq import heappush, heappop
import itertools
import math
import time

class myPlan():

    def __init__(self, worldMap=[], costMap=[], poseX=0, poseY=0, dim=10 ):
        self.occupMap = worldMap
        self.costMap = costMap
        self.visitMap = np.ones([dim,dim])
        self.IG_Map = 9*np.ones([dim,dim])*(self.occupMap+1)
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
        
        
        

        self.UGVPath = np.array([])
        self.UAVPath = np.array([])
        
        
        self.deploy_ = 0
        self.return_ = False
        
        
        ### tunning parameter
        self.paradist = 0.3
        self.paraig = 1 - self.paradist
        self.penalty = 0.0001 #0.3  0.8 0.01
        self.m_block = 2
        self.AstarWeight = 1.05

        self.globalTimeThreshold = 200
        self.batteryLife = 100
        self.batteryCapacity = 100
        
        self.IGacc = 0
        self.iglist = np.array([])
        
        self.visit = 0
        self.visitlist = np.array([])
        
    
        
        ### end tunning para
        
        #np.savetxt('costMap.txt', costMap, '%d')
        self.IGsum = []
        
                
        
    def generateDistMap(self, X,Y, isUAV = False):
        # run Dijkstra
        A_closelist = {}
        parent= {}
        self.PQ = priorityQ()
        self.PQ.add_task(self.pix2ind([Y, X]),0,0)
        while(len(self.PQ.pq)>0):
            priority,g, key = self.PQ.pop_task()
            
           
            A_closelist[key] = g 
            
            pos  = self.ind2pix(key)
            y = pos[0]
            x = pos[1]
            if isUAV:
                self.distMap4UAV[y,x] = g-1
            else:
                self.distMap4UGV[y,x] = g-self.costMap[y,x]
            
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

        self.PQ=priorityQ()
        A_closelist = {}
        parent= {}
        self.PQ.add_task(self.pix2ind([startY, startX]),0,0)
        
        while(len(self.PQ.pq)>0):
            priority,g, key = self.PQ.pop_task()

            # if startX ==11 and startY ==2:        
            #     print('xxx',key)
            A_closelist[key] = g 

            # if find goal, break the loop
            if key == self.pix2ind([goalY, goalX]):
                break
            
            # if startX ==11 and startY ==2:        
            #     print('xxx',startX, startY)
                
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
        # path.reverse()
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
        if new_pose :
            if not A_flag:
                heristic = 0
                if isUAV_flag:                    
                    cost = 1
                else:
                    cost = self.costMap[Y,X]
            else:
                # if use A*, herisitic gerenated from distMap
                if isUAV_flag:                    
                    heristic = self.distMap4UAV[Y,X] * self.AstarWeight
                    cost = 1
                else:
                    heristic = self.distMap4UGV[Y,X] * self.AstarWeight                    
                    pos = self.ind2pix(preTask)
                    cost = self.costMap[pos[0],pos[1]]
                    
            new_pose_ind = self.pix2ind(new_pose)
            if new_pose_ind not in closeList:
                if self.PQ.add_task(new_pose_ind, pre_g, cost, h = heristic):
                    parents[new_pose_ind] = preTask
                    # print(new_pose_ind)
                    # print('parents of 0: ',parents[0])
        
# --------------------------------------------------------------
#                      Hacked by Anqi
# --------------------------------------------------------------
        
    # def generateUGVScoreMap(self, IG_map = [], m_block = 0.,Dmax = 0, deploy_ = False ):
    def generateUGVScoreMap(self, Dmax = 0, flag = 1):
        #TODO use euclidean? or other matric?
    
        # IG_map = np.array(IG_map)
        # score = np.divide(IG_map, self.distMap4UGV) 
        score = np.divide(pow(self.IG_Map,self.paraig), pow(self.distMap4UGV+2,self.paradist))
        score = score*(1-flag*self.occupMap)
        
        # hack begin
        
        new_pose = self.checkBoarder([self.UGVY,self.UGVX])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UGVY+1, self.UGVX])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UGVY, self.UGVX+1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UGVY-1, self.UGVX])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UGVY, self.UGVX-1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UGVY+1, self.UGVX+1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UGVY+1, self.UGVX-1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UGVY-1, self.UGVX+1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UGVY-1, self.UGVX-1])
        if new_pose:
            score[new_pose] = 0
        
        # hack end
        
        
        
        # if deploy_ :
            # m_block = 1/m_block
        
        # score *= m_block


        if self.deploy_:
            for i in range(self.dim):
                for j in range(self.dim):
                    #if self.occupMap[j,i]:  # by Wenhao
                    #    continue            # by Wenhao
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

        score = np.divide(pow(self.IG_Map,self.paraig), pow(self.distMap4UAV+2,self.paradist))
        
        # hack begin
        new_pose = self.checkBoarder([self.UAVY,self.UAVX])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UAVY+1, self.UAVX])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UAVY, self.UAVX+1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UAVY-1, self.UAVX])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UAVY, self.UAVX-1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UAVY+1, self.UAVX+1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UAVY+1, self.UAVX-1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UAVY-1, self.UAVX+1])
        if new_pose:
            score[new_pose] = 0
        new_pose = self.checkBoarder([self.UAVY-1, self.UAVX-1])
        if new_pose:
            score[new_pose] = 0
        # hack end
        
        
        
        
        
        
        for i in range(self.dim):
            for j in range(self.dim):
                if self.distMap4UGV[j,i] <= Dmax:
                    score[j,i] *= 1
                else:
                    score[j,i] *= Dmax/self.distMap4UGV[j,i] * self.penalty
        
        score[self.UAVY, self.UAVX] = 0

        return score

    def UAVGoBackPlan(self, UGVPath, UAVLoc, batteryLevel):
        print("pre goback UAV location:",UAVLoc)
        print("UGVPath: ", UGVPath)
        print("batteryLevel back:", batteryLevel)
        r_step_ = 0
        for i in range(batteryLevel):
            if i < UGVPath.shape[0]:
                if sp.spatial.distance.chebyshev(UGVPath[i,:],UAVLoc) < i:
                    r_step_ = i
                    break
            else:
                if sp.spatial.distance.chebyshev(UGVPath[-1,:],UAVLoc) < i:
                    r_step_ = i
                    UGVPath = np.repeat(UGVPath,(UGVPath.shape[0]-1)*[1] +[r_step_-UGVPath.shape[0]+1],axis=0)
                    break
                if i == batteryLevel:
                    print('UAVGoBackPlan: wrong battery level!!!')
#         for i in range(UGVPath.shape[0]):
#             if np.max(np.abs(UGVPath[i,:]-UAVLoc)) <= batteryLevel:
#                 r_step_ = i
#                 break
#             # print(UGVPath.shape[0])
#             if i is UGVPath.shape[0]-1:
#                 r_step_ = np.max(np.abs(UGVPath[-1,:]-UAVLoc))
#                 #np.append(UGVPath,np.matlib.repmat(UGVPath[-1,:],r_step_-UGVPath.shape[0],1),axis=0)
#                 UGVPath = np.repeat(UGVPath,(UGVPath.shape[0]-1)*[1] +[r_step_-UGVPath.shape[0]+1],axis=0)
#                 # print(UGVPath.shape[0],r_step_)
#                 assert UGVPath.shape[0] == r_step_
#                 break

        r_loc_ = UGVPath[-1,:]
    
        x_ = UAVLoc
        path = np.zeros((r_step_,2))
        for i in range(r_step_):
            path[i,:] = x_
            if r_loc_[0] > x_[0]:
                x_[0] = x_[0]+1

            if r_loc_[0] < x_[0]:
                x_[0] = x_[0]-1

            if r_loc_[1] > x_[1]:
                x_[1] = x_[1]+1

            if r_loc_[1] < x_[1]:
                x_[1] = x_[1]-1

#        path.reverse()

        # print(path)

        
        UAVPath = np.copy(UGVPath)
        UAVPath[:r_step_,:] = path
        self.deploy_ = 0
        
        # print(UGVPath)
        # print(UAVPath)

        return UGVPath, UAVPath


    def updateVisitMap(self,UAVPath,UGVPath):
        if self.deploy_:
            self.visitMap[UAVPath[:,0],UAVPath[:,1]] += 1
            self.visitMap[UGVPath[:,0],UGVPath[:,1]] += 1
        else:
            self.visitMap[UGVPath[:,0],UGVPath[:,1]] += 1


    def updateIGMap(self):
        IG_Map_ = np.divide(((self.m_block-1)*self.occupMap+1),self.visitMap)
        # IG_Map_ = ((self.m_block-1)*self.occupMap+1)*np.exp(-self.visitMap)
        self.IG_Map = signal.convolve2d(IG_Map_,np.ones((3,3)),mode='same', boundary = 'symm')  
        
        # for summary:
        LL = len(self.IGsum)
        if LL == 0:
            self.IGsum.append(np.sum(self.IG_Map))
        else:
            last = self.IGsum[-1]
            #print(last)            # comment by Wenhao
            self.IGsum.append(np.sum(self.IG_Map)+last)
            

    def recordPath(self,UGVPath_,UAVPath_):
        #print(self.UGVPath.shape, UGVPath_.shape)

        path_len_ = min(UGVPath_.shape[0],UAVPath_.shape[0])
        if len(self.UGVPath) ==0 :
            self.UGVPath = UGVPath_[:path_len_,:]
            self.UAVPath = UAVPath_[:path_len_,:]
        else:
            self.UGVPath = np.append(self.UGVPath, UGVPath_[1:path_len_,:], axis=0)
            self.UAVPath = np.append(self.UAVPath, UAVPath_[1:path_len_,:], axis=0)
        assert self.UGVPath.shape[0] == self.UAVPath.shape[0]
        
        #np.savetxt('UGV.txt', self.UGVPath,'%d')
        #np.savetxt('UAV.txt', self.UAVPath,'%d')
        
    def getaccIG(self,UGVPath,UAVPath):
        #print(self.UGVPath.shape, UGVPath_.shape)
        
        if self.deploy_:
            self.IGacc += np.sum(self.IG_Map[UAVPath[:,0],UAVPath[:,1]])
            self.IGacc += np.sum(self.IG_Map[UGVPath[:,0],UGVPath[:,1]])
        else:
            self.IGacc += np.sum(self.IG_Map[UGVPath[:,0],UGVPath[:,1]])  
            
            
        #if len(self.iglist) ==0 :
         #   self.iglist = np.array(self.IGacc)
        #else:
        self.iglist = np.append(self.iglist, np.array(self.IGacc))
        
        
    def countVisit(self):
        ks = np.count_nonzero(self.visitMap-1)
        self.visit = np.divide(ks,self.dim*self.dim)
        self.visitlist = np.append(self.visitlist, np.array(self.visit))



    def nearBlock(self,UGVX,UGVY):
        for i in range(-1,2):
            for j in range(-1,2):
                if not self.checkBoarder([UGVY+j, UGVX+j]):
                    return False
                if self.occupMap[UGVY+i,UGVX+j]:
                    return True

        return False
        
        
    def execute(self):
        
        
        
        #while step < self.globalTimeThreshold:
            
        self.generateDistMap(self.UGVX, self.UGVY, isUAV = False)
        
        if self.deploy_:
            self.generateDistMap(self.UAVX, self.UAVY, isUAV = True)
    
        self.updateIGMap()
        
        if not self.deploy_:
            scoreMapUGV = self.generateUGVScoreMap(self.batteryLife * 0.9 , flag = 0)
            UGVFrontier = np.unravel_index(scoreMapUGV.argmax(), scoreMapUGV.shape)
            if self.occupMap[UGVFrontier[0],UGVFrontier[1]]:
                self.deploy_ = True
                print('deploying')
                print(self.batteryLife)
                scoreMapUGV = scoreMapUGV*(1-self.occupMap)
                UGVFrontier = np.unravel_index(scoreMapUGV.argmax(), scoreMapUGV.shape)
        else:
            scoreMapUGV = self.generateUGVScoreMap(self.batteryLife * 0.9 )
            UGVFrontier = np.unravel_index(scoreMapUGV.argmax(), scoreMapUGV.shape)
        
        # print(UGVFrontier)
        # print(self.UGVX,self.UGVY)
        UGVPath = self.runAstar(UGVFrontier[1], UGVFrontier[0], self.UGVX,self.UGVY, isUAV = False)
        # print(UGVPath)

        if self.deploy_:
            scoreMapUAV = self.generateUAVScoreMap(self.batteryLife * 0.8 )
            UAVFrontier = np.unravel_index(scoreMapUAV.argmax(), scoreMapUAV.shape)
            if sp.spatial.distance.chebyshev(np.array(UAVFrontier), np.array(UGVFrontier)) < self.batteryLife * 0.5:
                UAVPath =  self.runAstar(UAVFrontier[1],UAVFrontier[0], self.UAVX, self.UAVY, isUAV = True)
            else:
                print(self.UGVX,self.UGVY, UGVFrontier)
                print(self.UAVX,self.UAVY)
                UGVPath, UAVPath = self.UAVGoBackPlan(UGVPath, np.array([self.UAVY, self.UAVX]), self.batteryLife)
                self.return_ = True
        
        else:
            UAVPath = UGVPath
    
        # TODO: will UAVPath be zero length?
    
        minTimeStep  = min(len(UAVPath),len(UGVPath))
       # step += minTimeStep-1
        
      #  print(step)
        
        
        self.updateVisitMap(UAVPath[0:minTimeStep,:], UGVPath[0:minTimeStep,:])
        self.UAVX = UAVPath[minTimeStep-1,1]
        self.UAVY = UAVPath[minTimeStep-1,0]
        self.UGVX = UGVPath[minTimeStep-1,1]
        self.UGVY = UGVPath[minTimeStep-1,0]
        
        #print(minTimeStep, len(UGVPath), len(UAVPath))            
        self.recordPath( UGVPath[0:(minTimeStep),:], UAVPath[0:(minTimeStep),:])
        
        self.getaccIG(UGVPath[0:(minTimeStep),:], UAVPath[0:(minTimeStep),:])
        self.countVisit()
        
        np.savetxt('../results/IGlist.txt',self.iglist,'%.4f')
        np.savetxt('../results/visitlist.txt',self.visitlist,'%.4f')
    
        
        if self.deploy_:
            self.batteryLife -= minTimeStep
            
        if self.nearBlock(self.UGVX,self.UGVY) and not self.return_ and not self.deploy_:
            self.deploy_ = True
            print('deploying near')
            print(self.batteryLife)
            
        if self.return_ == True:
            self.batteryLife = self.batteryCapacity
            self.return_ = False
            self.deploy_ = False            
            
            
        if self.batteryLife <= 0:
            print('wrong batteryLife')                
       #     break
            
        #np.savetxt('visit.txt', self.visitMap,'%d')
        #np.savetxt('score.txt', scoreMapUGV, '%.4f')
                
    def summarize(self):
        np.savetxt('../results/IGsum.txt',np.array(self.IGsum),'%.4f')
        #np.savetxt('../results/IGmap.txt',self.IG_Map,'%.4f')
            
            
            
            
        
        
        

        
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
        return True

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


# --------------------------------------------------------------
#                      Anqi's code begins here
# --------------------------------------------------------------



# --------------------------------------------------------------
#                      Anqi's code ends here
# --------------------------------------------------------------


#a.runAstar(0,0,20,20)