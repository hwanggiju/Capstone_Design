from re import L
import pygame
import math
import sys
from mpu9250_i2c import *
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

time.sleep(1)

class Cube(object) :
    def __init__(self, position, color):
        self.position = position
        self.color = color
        
    #
    num_faces = 6
    
    vertices = [ (-1.0, -0.05, 0.5),
                (1.0, -0.05, 0.5),
                (1.0, 0.05, 0.5),
                (-1.0, 0.05, 0.5),
                (-1.0, -0.05, -0.5),
                (1.0, -0.05, -0.5),
                (1.0, 0.05, -0.5),
                (-1.0, 0.05, -0.5) ]
    
    normals = [ (0.0, 0.0, 1.0),        # 앞
                (0.0, 0.0, -1.0),       # 뒤
                (1.0, 0.0, 0.0),        # 오른
                (-1.0, 0.0, 0.0),       # 왼
                (0.0, 1.0, 0.0),        # 위
                (0.0, -1.0, 0.0) ]      # 아래
    
    vertex_indices = [ (0, 1, 2, 3),    # 앞
                       (4, 5, 6, 7),    # 뒤
                       (1, 5, 6, 2),    # 오른
                       (0, 4, 7, 3),    # 왼
                       (3, 2, 6, 7),    # 위
                       (0, 1, 5, 4)]    # 아래
    
    def render(self) :
        then = pygame.time.get_ticks()
        glColor(self.color)
        
        vertices = self.vertices
        
        glBegin(GL_QUADS)
        
        for face_no in range(self.num_faces) :
            glNormal3dv(self.normals[face_no])
            v1, v2, v3, v4 = self.vertex_indices[face_no]
            glVertex(vertices[v1])
            glVertex(vertices[v2])
            glVertex(vertices[v3])
            glVertex(vertices[v4])
        glEnd()
        
def dist(a, b) :
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x, y, z) :
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)

def get_x_rotation(x, y, z) :
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

SCREEN_SIZE = (800, 600)
SCALAR = .5
SCALAR2 = 0.2

def resize(width, height) :
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(width) / height, 0.001, 10.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0.0, 1.0, -5.0,
              0.0, 0.0, 0.0,
              0.0, 1.0, 0.0)
               
def init() :
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_BLEND)
    glEnable(GL_POLYGON_SMOOTH)
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)
    glEnable(GL_COLOR_MATERIAL)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHTO)
    glLightfv(GL_LIGHTO, GL_AMBIENT, (0.3, 0.3, 0.3, 1.0));
    
# main
pygame.init()
screen = pygame.display.set_mode(SCREEN_SIZE, HWSURFACE | OPENGL | DOUBLEBUF)
resize(*SCREEN_SIZE)
init()
clock = pygame.time.Clock()
cube = Cube((0.0, 0.0, 0.0), (255., 0., 0.))
angle = 0

print('데이터 기록')
while True :
    try :
        ax, ay, az = MPU6050_raw_data() 
        
    except :
        continue
    
    accel_xout_scaled = ax / 16384.0
    accel_yout_scaled = ay / 16384.0
    accel_zout_scaled = az / 16384.0
       
    then = pygame.time.get_ticks()
    for event in pygame.event.get():
        if event.type == QUIT:
            sys.exit(1)
        if event.type == KEYUP and event.key == KEY_ESCAPE :
            sys.exit(1)
            
    # mpu6050 데이터 가공
    x_angle = get_x_rotation(accel_xout_scaled, accel_yout_scaled, \
                            accel_zout_scaled)
    y_angle = get_y_rotation(accel_xout_scaled, accel_yout_scaled, \
                            accel_zout_scaled)

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    
    glColor((1., 1., 1.))
    glLineWidth(1)
    glBegin(GL_LINES)
    
    for x in range(-20, 22, 2) :
        glVertex3f(x/10., -1, -1)
        glVertex3f(x/10., -1, 1)
        
    for x in range(-20, 22, 2) :
        glVertex3f(x/10., -1, 1)
        glVertex3f(x/10., 1, 1)
        
    for z in range(-10, 12, 2) :
        glVertex3f(-2, -1, z/10.)
        glVertex3f(2, -1, z/10.)
        
    for z in range(-10, 12, 2) :
        glVertex3f(-2, -1, z/10.)
        glVertex3f(-2, 1, z/10.)
        
    for z in range(-10, 12, 2) :
        glVertex3f(2, -1, z/10.)
        glVertex3f(2, 1, z/10.)
        
    for y in range(-10, 12, 2) :
        glVertex3f(-2, y/10., 1)
        glVertex3f(2, y/10., 1)
        
    for y in range(-10, 12, 2) :
        glVertex3f(-2, y/10., 1)
        glVertex3f(-2, y/10., -1)
        
    for y in range(-10, 12, 2) :
        glVertex3f(2, y/10., 1)
        glVertex3f(2, y/10., 1) 
        
    glEnd()
    glPushMatrix()
    glRotate(float(x_angle), 1, 0, 0)
    glRotate(-float(y_angle), 0, 0, 1)
    cube.render()
    glPopMatrix()
    pygame.display.filp()