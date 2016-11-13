# -*- coding: utf-8 -*-
"""
Created on Sun Nov 13 14:44:41 2016

Planning Course Project

@author: chiyud
"""

from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt4 import QtGui,QtCore
from PyQt4.QtOpenGL import *

class MainWindow(QtGui.QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.widget = glWidget(self)

        #self.button = QtGui.QPushButton('Test', self)

        mainLayout = QtGui.QHBoxLayout()
        mainLayout.addWidget(self.widget)
        #mainLayout.addWidget(self.button)

        self.setLayout(mainLayout)




class glWidget(QGLWidget):

   
    
    def __init__(self, parent):
        QGLWidget.__init__(self, parent)
        self.setMinimumSize(640, 480)
        self.x = 0
        self.y = 0
        self.z = 0
   
    def paintGL(self):


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()


        #glTranslatef(-2.5, 0.5, -6.0)
        glColor3f( 1.0, 1.5, 0.0 );
        glPolygonMode(GL_FRONT, GL_FILL);

        glBegin(GL_TRIANGLES)
        glVertex3f(self.x+2,    self.y-1.2, self.z+ 0.0)
        glVertex3f(self.x+2.6,  self.y+0.0, self.z+ 0.0)
        glVertex3f(self.x+2.9,  self.y-1.2, self.z+ 0.0)
        glEnd()
        
        self.DrawGround()

        glFlush()



    def initializeGL(self):



        glClearDepth(1.0)              
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()                    
        gluPerspective(45.0,1.33,0.1, 100.0) 
        gluLookAt(0,0, 50,   0, 0, 0,   0, 1, 0); 

        glMatrixMode(GL_MODELVIEW)


    def DrawGround(self):
        glColor3f (0.3, 0.3, 0.3);
        glBegin(GL_LINES);
        area = 50        
        for i in range(-area,area):
            glVertex3f(i, area, 0); glVertex3f(i, -area, 0);
            glVertex3f(area, i, 0); glVertex3f(-area, i, 0);        
        
        glEnd();
        
    def mousePressEvent(self, event):
        
        self.lastPos = QtCore.QPoint(event.pos())
        self.x +=1
        self.y +=1
        self.updateGL()
        
        print(self.x,self.y)
        
    def moveObject(self,event):
        pass


if __name__ == '__main__':
    app = QtGui.QApplication(['Planning'])
    window = MainWindow()
    window.show()
    app.exec_()