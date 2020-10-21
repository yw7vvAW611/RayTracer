import sys
from PIL import Image
import math
import numpy as np

with open(sys.argv[1], 'r') as f:
    contents = f.read().splitlines()

try:
    line = contents[0].split()
    contents.remove(contents[0])
    width = int(line[1])
    height = int(line[2])
    filename = line[3]
    img = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    putpixel = img.im.putpixel
except:

    print('First line is invalid input')

class vec3():
    def __init__(self, x, y, z):
        (self.x, self.y, self.z) = (x, y, z)
    def __mul__(self, num):
        return vec3(self.x * num, self.y * num, self.z * num)
    def __add__(self, num):
        return vec3(self.x + num.x, self.y + num.y, self.z + num.z)
    def __sub__(self, num):
        return vec3(self.x - num.x, self.y - num.y, self.z - num.z)
    def dot(self, vec):
        return (self.x * vec.x) + (self.y * vec.y) + (self.z * vec.z)
    def __abs__(self):
        return self.dot(self)
    def norm(self):
        mag = np.sqrt(abs(self))
        return self * (1.0 / np.where(mag == 0, 1, mag))
    def components(self):
        return (self.x, self.y, self.z)

class Ray(object):
    def __init__(self, x, y):
        self.x = x
        self.y =y
        self.d = vec3(0,0,0)
        self.origin = origin

    def scalar(self):
        s_x = (2 * self.x - width) / max(width,height)
        s_y = (height - 2 * self.y) /max(width,height)
        return s_x, s_y

    def dir(self):
        s_x,s_y = self.scalar()
        self.d = f + r * s_x + u * s_y
        return self.d

    def src(self):
        return self.origin

def hit_sphere(center, r, ray):
    oc = ray.src() - center
    a = ray.dir().dot(ray.dir())
    b = 2 * ray.dir().dot(oc)
    c = oc.dot(oc) - r * r
    return b * b - 4 * a * c > 0

def sphereProcess(line):
    index1 = float(line[1])
    index2 = float(line[2])
    index3 = float(line[3])
    r = float(line[4])
    center = vec3(index1,index2, index3)
    if len(colorList) == 0:
        color = (0 , 0, 0)
    else:
        color = colorList[-1]
    color = colorTrans(color)
    for i in range(width):
        for j in range(height):
            ray = Ray(i , j)
            if hit_sphere(center , r, ray):
                putpixel((i, j), color)
    return


def colorProcess(line):
    c = line[1:]
    # print(c, "list")
    for i in c:
        t = float(i)
        if t <= 0:
            t = 0
        elif t >= 1:
            t = 1
        colorList.append(t)
    return colorList

def colorTrans(c):
    color = []
    for i in c:
        color.append(round(i * 255))
    return tuple(color)

origin = vec3(0,0,0)
f = vec3(0,0,-1)
r = vec3(1,0,0)
u = vec3(0,1,0)
colorList = []

for line in contents:
    l = line.split()
    if len(l) > 0:
        keyword = l[0]
        if keyword == 'sphere':
            sphereProcess(l)
        elif keyword == 'color':
            colorProcess(l)
        #     vertexLists.append(l)
        # elif keyword == 'trif':
        #     trifProcess(vertexLists, colorList,l)
        # elif keyword == 'color':
        #     l = colorProcess(l)
        #     colorList.append(l)
        # elif keyword == 'loadmv':
        #     MV = loadmvProcess(l)
        # elif keyword =='translate':
        #     MV = translateProcess(l)
        # elif keyword == 'rotatex' or keyword=='rotatey' or keyword == 'rotatez':
        #     MV = rotatexProcess(l)
        # elif keyword == 'scale':
        #     scaleProcess(l)
        # elif keyword == 'loadp':
        #     PM = loadmvProcess(l)
        # elif keyword == 'multmv':
        #     MV = multmvProcess(l)
        # elif keyword == 'trig':
        #     trigProcess(vertexTrig,l)
        # elif keyword =='rotate':
        #     MV = rotateProcess(l)
img.save(filename)