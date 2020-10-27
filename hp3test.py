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
    def __truediv__(self, num):
        return vec3(self.x  / num, self.y / num, self.z / num)

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

    def point_at_t(self, t):
        return self.origin + self.d * t

class HitPoint(object):
    def __init__(self, ray, obj, t, point, normal):
        self.ray = ray
        self.obj = obj
        self.t = t
        self.point = point
        self.normal = normal


class Sphere(object):
    def __init__(self, center, r):
        self.center = center
        self.r = r

    def find_hit_point(self, ray):
        oc = ray.src() - self.center
        a = ray.dir().dot(ray.dir())
        b = ray.dir().dot(oc)
        c = oc.dot(oc) - self.r * self.r
        discriminant = b * b - a * c

        hit_points = []

        if discriminant > 0:
            t1 = (- b - math.sqrt(discriminant)) / a
            p1 = ray.point_at_t(t1)
            n1 = (p1 - self.center) / self.r
            t2 = (- b + math.sqrt(discriminant)) / a
            p2 = ray.point_at_t(t2)
            n2 = (p2 - self.center) / self.r
            hit_points.append(HitPoint(ray, self, t1, p1, n1))
            hit_points.append(HitPoint(ray, self, t2, p2, n2))

        return hit_points


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
    sphere = Sphere(center, r)
    obj_list.append(sphere)
    # if len(colorList) == 0:
    #     color = (0 , 0, 0)
    # else:
    #     color = colorList[-1]
    # color = colorTrans(color)
    # for i in range(width):
    #     for j in range(height):
    #         ray = Ray(i , j)
    #         if hit_sphere(center , r, ray):
    #             putpixel((i, j), color)
    return

def colorObject(obj_list, ray):
    if len(colorList) == 0:
        object_color = (1, 1, 1)
    else:
        object_color = colorList[-1]
    object_color = colorTrans(object_color)
    if len(light) > 0:
        light_dir = light[-1]
    else:
        light_dir = None

    for sphere in obj_list:
        hit_points = sphere.find_hit_point(ray)
        hit_points = list(filter(lambda x: x.t > 0.001, hit_points))
        if hit_points:
            hit_points.sort(key=lambda x: x.t)
            if light_dir == None:
                return object_color
            # else:
            #     n = hit_points[0].normal
            #     t = vec3(0,0,0).dot(vec3(1,1,1))
            #     t = t * (light_dir.dot(n))
            #     print(t)
            #     color = (t.x, t.y, t.z)
            #     return color


        # if hit_sphere(sphere.center, sphere.r, ray):
        #     if light_color == None:
                return object_color
    return None

class HitPoint(object):
    def __init__(self, ray, obj, t, point, normal):
        self.ray = ray
        self.obj = obj
        self.t = t
        self.point = point
        self.normal = normal

def sunProcess(line):
    x = float(line[1])
    y = float(line[2])
    z = float(line[3])
    lightDir = vec3(x , y, z)
    light.append(lightDir)


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
light = []
obj_list = []
for line in contents:
    l = line.split()
    if len(l) > 0:
        keyword = l[0]
        if keyword == 'sphere':
            sphereProcess(l)
        elif keyword == 'color':
            colorProcess(l)
        elif keyword == 'sun':
            sunProcess(l)
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

for i in range(width):
    for j in range(height):
        ray = Ray(i , j)
        color = colorObject(obj_list, ray)
        if color != None:
            putpixel((i, j), color)

img.save(filename)