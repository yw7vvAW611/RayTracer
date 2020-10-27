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

    def __mul__(self, other):
        return vec3(self.x * other.x, self.y * other.y, self.z * other.z)

    def __add__(self, other):
        return vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __truediv__(self, num):
        return vec3(self.x  / num, self.y / num, self.z / num)

    def scale(self, other):
        return vec3(self.x * other, self.y * other, self.z * other)

    def dot(self, vec):
        return (self.x * vec.x) + (self.y * vec.y) + (self.z * vec.z)

    def __abs__(self):
        return self.dot(self)

    def norm(self):
        mag = np.sqrt(abs(self))
        return self * (1.0 / np.where(mag == 0, 1, mag))

    def toTuple(self):
        return (self.x, self.y, self.z)

    def toList(self):
        return [self.x, self.y, self.z]

    def __repr__(self):
        return "x:% s y:% s z:%s" % (self.x, self.y, self.z)



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
        self.d = f + r.scale(s_x) + u.scale(s_y)
        return self.d

    def src(self):
        return self.origin

    def point_at_t(self, t):
        return self.origin + self.d.scale(t)

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


def sphereProcess(line):
    index1 = float(line[1])
    index2 = float(line[2])
    index3 = float(line[3])
    r = float(line[4])
    center = vec3(index1,index2, index3)
    sphere = Sphere(center, r)
    object_list.append(sphere)
    if len(colorList) == 0:
        object_color.append(vec3(1, 1, 1))
    else:
        object_color.append(colorList[-1])

def colorProcess(line):
    c = line[1:]
    color = []
    for i in c:
        t = float(i)
        if t <= 0:
            t = 0
        elif t >= 1:
            t = 1
        color.append(t)
    r, g, b = c
    colorList.append(vec3(r,g, b))
    return colorList

def sunProcess(line):
    x = float(line[1])
    y = float(line[2])
    z = float(line[3])
    lightDir = vec3(x , y, z)
    light_position.append(lightDir)
    light_color.append(colorList[-1])


object_list = []
object_color = []
light_position = []
light_color = []
colorList = []
origin = vec3(0,0,0)
f = vec3(0,0,-1)
r = vec3(1,0,0)
u = vec3(0,1,0)

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



# print(light_color)
# print(light_position)
# print(object_color)
# print(object_list)
def colorTrans(c):
    color = []
    for i in c:
        color.append(round(i * 255))
    return tuple(color)

def colorObject(ray):
    for i in range(len(object_list)):
        object = object_list[i]
        o_color = object_color[i]
        hit_points = object.find_hit_point(ray)
        hit_points = list(filter(lambda x: x.t > 0.001, hit_points))
        if len(light_position)==0:
            l_direction = vec3(0,0,0)
            l_color = vec3(0,0,0)
        else:
            l_direction = light_position[0]
            l_color = light_color[0]
        if hit_points:
            hit_points.sort(key=lambda x: x.t)
            hit_point = hit_points[0]
            color = (o_color * l_color).scale(l_direction.dot(hit_point.normal))
            color = colorTrans(color.toList())
            return color



for i in range(width):
    for j in range(height):
        ray = Ray(i , j)
        color = colorObject(ray)
        if color != None:
            putpixel((i, j), color)



img.save(filename)