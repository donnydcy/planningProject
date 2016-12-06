# -*- coding: utf-8 -*-
"""
Created on Sun Nov 13 14:44:41 2016

Planning Course Project

@author: chiyud
"""

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt4 import QtGui,QtCore
from PyQt4.QtOpenGL import *
from PIL.Image import open
import numpy as np
from time import sleep
from plan import *

# path to map files
GROUND_MAP = '../data/map_one.txt'
AERIAL_MAP = '../data/map_one.txt'


class Leaf3DPose():
    def __init__(self,X=0,Y=0,Z=0,RotX =0, RotY = 0, RotZ =0):
        self.x = X
        self.y = Y
        self.z = Z
        self.rotX = RotX
        self.rotY = RotY
        self.rotZ = RotZ

class MainWindow(QtGui.QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.widget = glWidget(self)


        mainLayout = QtGui.QHBoxLayout()
        mainLayout.addWidget(self.widget)

        self.setLayout(mainLayout)

class Map:
    def __init__(self,width,height,data):
        self.width = width
        self.height = height
        self.data = data


class glWidget(QGLWidget):

   
    
    def __init__(self, parent):
        QGLWidget.__init__(self, parent)
        self.setMinimumSize(640, 480)
        self.ugvX = 2
        self.ugvY = 11
        self.ugvZ = 0

        self.uavX = 2
        self.uavY = 11
        self.uavZ = 20

        self.ugvPath = [[self.ugvX,self.ugvY,self.ugvZ]]
        self.uavPath = [[self.uavX,self.uavY,self.uavZ]]

        width,height,data = self.loadMap(GROUND_MAP)
        self.groundMap = Map(width,height,data)

        width,height,data = self.loadMap(AERIAL_MAP)
        self.aerialMap = Map(width,height,data)

        self.cameraX = -width/2
        self.cameraY = -height/2
        self.cameraZ = 50

        self.rotX = 0
        self.rotY = 0
        self.rotZ = 0

        assert self.groundMap.width == self.aerialMap.width
        assert self.groundMap.height == self.aerialMap.height

        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        
      
        
        cost = np.zeros([100,100])
        self.a = myPlan(np.array(self.groundMap.data).transpose(), np.array(self.groundMap.data).transpose()*499+1)
        # self.a.generateDistMap(12,1)
        # ugvPath = self.a.runAstar(0,0,1,1)
        # print(ugvPath)
        # self.ugvPath = [[node[1],node[0]] for node in ugvPath]
        # self.uavPath = [[node[1],node[0]] for node in ugvPath]
        # self.updateGL()
        # self.moveObject()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.moveObject)
        interval = 1000.0 / 50.0
        self.timer.start( interval )
        print('initialization done')
  
    def paintGL(self):


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        glPushMatrix()

        glTranslatef(self.cameraX,self.cameraY,self.cameraZ-200)
        # glTranslatef(0,10,self.cameraZ-50)
        
        glRotatef(self.rotX,1,0,0);
        glRotatef(self.rotY,0,1,0);
        glRotatef(self.rotZ,0,0,1);
        # glRotatef(15+self.cameraZ-50,0,0,1)
        glColor3f( 1.0, 1.0, 0.0 );
        glPolygonMode(GL_FRONT, GL_FILL);


        
        glBegin(GL_QUADS)
        glVertex3f(self.ugvX-0.5, self.ugvY-0.5, self.ugvZ)
        glVertex3f(self.ugvX+0.5, self.ugvY-0.5, self.ugvZ)
        glVertex3f(self.ugvX+0.5, self.ugvY+0.5, self.ugvZ)
        glVertex3f(self.ugvX-0.5, self.ugvY+0.5, self.ugvZ)
        glEnd()

        glBegin(GL_QUADS)
        glVertex3f(self.uavX-0.5, self.uavY-0.5, self.uavZ)
        glVertex3f(self.uavX+0.5, self.uavY-0.5, self.uavZ)
        glVertex3f(self.uavX+0.5, self.uavY+0.5, self.uavZ)
        glVertex3f(self.uavX-0.5, self.uavY+0.5, self.uavZ)
        # glVertex3f(self.x-2.6,  self.y+0.0, self.z)
        # glVertex3f(self.x-2.9,  self.y-1.2, self.z)
        glEnd()
        
        self.DrawFootprint()
        self.DrawObstacles()
        self.DrawGround()
        
        glPopMatrix()
              
        
        glFlush()



    def initializeGL(self):



        glClearDepth(1.0)              
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glEnable (GL_BLEND)
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()                    
        gluPerspective(45.0,1.33,0.1, 800.0) 
        
     
                   
        gluLookAt(0,0 ,self.cameraZ,   0, 0, 0,   0, 1, 0); 
        glMatrixMode(GL_MODELVIEW)
        
        




    def DrawGround(self):
        glColor3f (0.3, 0.3, 0.3);
        glBegin(GL_LINES);
        area = 50        
        for i in range(0,self.groundMap.width,2):
            glVertex3f(i, 0, 0); glVertex3f(i, self.groundMap.height, 0);
        for i in range(0,self.groundMap.height,2):
            glVertex3f(0, i, 0); glVertex3f(self.groundMap.width, i, 0);        
        
        glEnd();
        glColor4f(0.3,0.3,0.3,0.2)
        glBegin(GL_QUADS)
        glVertex3f(0,0,0)
        glVertex3f(self.groundMap.width,0,0)
        glVertex3f(self.groundMap.width,self.groundMap.height,0)
        glVertex3f(0,self.groundMap.height,0)
        glEnd()

        glColor4f(0.8,0.8,1,0.2)
        glBegin(GL_QUADS)
        glVertex3f(0,0,20)
        glVertex3f(self.groundMap.width,0,20)
        glVertex3f(self.groundMap.width,self.groundMap.height,20)
        glVertex3f(0,self.groundMap.height,20)
        glEnd()


    def DrawObstacles(self):
        # show obstacles in ground map in white
        [self.DrawPatch(i,j,0.0,[1,1,1]) for i in range(self.groundMap.width) for j in range(self.groundMap.height) if self.groundMap.data[i][j]]
        # show obstacles in aerial map in blue
        [self.DrawPatch(i,j,20,[0.5,0.5,1]) for i in range(self.aerialMap.width) for j in range(self.aerialMap.height) if self.aerialMap.data[i][j]]

    def DrawFootprint(self):
        [self.DrawPatch(loc[0],loc[1],0,[0,1,0]) for loc in self.ugvPath]
        [self.DrawPatch(loc[0],loc[1],20,[1,0.5,0]) for loc in self.uavPath]

    # draw individual patches, used by drawObstacles
    def DrawPatch(self,x,y,z,color):
        glPolygonMode(GL_FRONT, GL_FILL)
        glColor4f(color[0],color[1],color[2],0.8)
        glBegin(GL_QUADS)
        glVertex3f(x-0.5,y-0.5, z)
        glVertex3f(x+0.5,y-0.5, z)
        glVertex3f(x+0.5,y+0.5, z)
        glVertex3f(x-0.5,y+0.5, z)
        glEnd()
        
        
    def mousePressEvent(self, event):
        # self.moveObject()
        # self.lastPos = QtCore.QPoint(event.pos())
        # self.ugvX +=1
        # self.ugvY +=1
        # self.uavX +=1
        # self.uavY +=1
        # self.ugvPath.append([self.ugvX,self.ugvY,self.ugvZ])
        # self.uavPath.append([self.uavX,self.uavY,self.uavZ])
        # self.updateGL()
        pass


        
    def wheelEvent(self,event):
        d = event.delta()
        self.cameraZ += (d and d // abs(d))
        self.updateGL()

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.close()
        else:
            if event.key() == QtCore.Qt.Key_Up:
                self.cameraY += 2
            if event.key() == QtCore.Qt.Key_Down:
                self.cameraY -= 2
            if event.key() == QtCore.Qt.Key_Left:
                self.cameraX -= 2
            if event.key() == QtCore.Qt.Key_Right:
                self.cameraX += 2

            if event.key() == QtCore.Qt.Key_W:
                self.rotX -= 2
            if event.key() == QtCore.Qt.Key_S:
                self.rotX += 2
            if event.key() == QtCore.Qt.Key_A:
                self.rotZ += 2
            if event.key() == QtCore.Qt.Key_D:
                self.rotZ -= 2

            if event.key() == QtCore.Qt.Key_Space:
                self.cameraZ += 2
            if event.key() == QtCore.Qt.Key_Return:
                self.cameraZ -= 2

            self.updateGL()
              
        
    def moveObject(self):
        self.a.execute()

        print('execute done')
        # print(self.a.UGVPath)
        self.ugvX = self.a.UGVX
        self.ugvY = self.a.UGVY
        self.uavX = self.a.UAVX
        self.uavY = self.a.UAVY
        self.ugvPath.append([self.ugvX,self.ugvY,self.ugvZ])
        
        self.uavPath.append([self.uavX,self.uavY,self.uavZ])
        self.updateGL()

        

    def loadMap(self, mapFile):
        mapData = np.loadtxt(mapFile)
        width = mapData.shape[0]
        height = mapData.shape[1]
        data = mapData.tolist()
        return width,height,data


if __name__ == '__main__':
    app = QtGui.QApplication(['Planning'])
    window = MainWindow()
    window.show()
    app.exec_()