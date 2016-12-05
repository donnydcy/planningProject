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

# path to map files
GROUND_MAP = '../data/map_1_obstacle_ground.txt'
AERIAL_MAP = '../data/map_1_obstacle_aerial.txt'


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
        self.x = 0
        self.y = 0
        self.z = 0

        self.path = [[self.x,self.y,self.z]]
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
        glVertex3f(self.x-0.5, self.y-0.5, self.z)
        glVertex3f(self.x+0.5, self.y-0.5, self.z)
        glVertex3f(self.x+0.5, self.y+0.5, self.z)
        glVertex3f(self.x-0.5, self.y+0.5, self.z)
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
        for i in range(self.groundMap.width):
            glVertex3f(i, 0, 0); glVertex3f(i, self.groundMap.height, 0);
        for i in range(self.groundMap.height):
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
        glVertex3f(0,0,5)
        glVertex3f(self.groundMap.width,0,5)
        glVertex3f(self.groundMap.width,self.groundMap.height,5)
        glVertex3f(0,self.groundMap.height,5)
        glEnd()


    def DrawObstacles(self):
        # show obstacles in ground map in white
        [self.DrawPatch(i,j,0.0,[1,1,1]) for i in range(self.groundMap.width) for j in range(self.groundMap.height) if self.groundMap.data[i][j]]
        # show obstacles in aerial map in blue
        [self.DrawPatch(i,j,5.0,[0.5,0.5,1]) for i in range(self.aerialMap.width) for j in range(self.aerialMap.height) if self.aerialMap.data[i][j]]

    def DrawFootprint(self):
        [self.DrawPatch(loc[0],loc[1],loc[2],[0,1,0]) for loc in self.path]

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
        
        self.lastPos = QtCore.QPoint(event.pos())
        self.x +=1
        self.y +=1
        self.path.append([self.x,self.y,self.z])
        self.updateGL()
        


        print(self.x,self.y)
        
    def wheelEvent(self,event):
        d = event.delta()
        self.cameraZ += (d and d // abs(d))
        self.updateGL()

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.close()
        else:
            if event.key() == QtCore.Qt.Key_Up:
                self.cameraY -= 2
            if event.key() == QtCore.Qt.Key_Down:
                self.cameraY += 2
            if event.key() == QtCore.Qt.Key_Left:
                self.cameraX += 2
            if event.key() == QtCore.Qt.Key_Right:
                self.cameraX -= 2

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
              
        
    def moveObject(self,event):
        pass

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