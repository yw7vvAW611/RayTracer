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
        (self.x, self.y, self.z) = (float(x), float(y), float(z))

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
        return self.scale (1.0 / np.where(mag == 0, 1, mag))

    def toTuple(self):
        return (self.x, self.y, self.z)

    def toIntTuple(self):
        return (round(self.x), round(self.y), round(self.z))

    def toList(self):
        return [self.x, self.y, self.z]

    def cross(self, b):
      return vec3(self.y * b.z - b.y * self.z,
                  self.z * b.x - b.z * self.x,
                  self.x * b.y - b.x * self.y)

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

class fishRay(object):
    def __init__(self, x, y):
        self.x = x
        self.y =y
        self.d = vec3(0,0,0)
        self.origin = origin

    def scalar(self):
        s_x = (2 * self.x - width) / max(width,height)
        s_y = (height - 2 * self.y) /max(width,height)
        global f
        f_length = math.sqrt(f.dot(f))
        s_x /= f_length
        s_y /= f_length
        f = f.norm()
        return s_x, s_y

    def dir(self):
        s_x,s_y = self.scalar()
        r = self.get_r()
        self.d = f.scale(math.sqrt(1-r*r)) + r.scale(s_x) + u.scale(s_y)
        return self.d

    def get_r(self):
        s_x, s_y = self.scalar()
        return math.sqrt(s_x*s_x + s_y*s_y)

    def src(self):
        return self.origin

    def point_at_t(self, t):
        return self.origin + self.d.scale(t)


class HitPoint(object):
    def __init__(self, ray, obj, t, point, normal,color):
        self.ray = ray
        self.obj = obj
        self.t = t
        self.point = point
        self.normal = normal
        self.color = color


class Sphere(object):
    def __init__(self, center, r, color):
        self.center = center
        self.r = r
        self.color = color

    def find_hit_point(self, ray):
        # hit_points = []
        # oc = self.center - ray.src()
        # r_2 = self.r * self.r
        # inside = True
        # if oc.dot(oc) < r_2:
        #     inside = False
        # a = ray.dir().dot(ray.dir())
        # t_c = oc.dot(ray.dir())/math.sqrt(a)
        # if t_c < 0 and inside:
        #     return hit_points
        # b = ray.src() + ray.dir().scale(t_c)
        # b = b - self.center
        # d_2 = b.dot(b)
        # if (d_2 > self.r * self.r) and inside:
        #     return hit_points
        # t_offset = math.sqrt(r_2-d_2)/math.sqrt(a)
        # t1 = t_c + t_offset
        # p1 = ray.point_at_t(t1)
        # n1 = (p1 - self.center) / self.r
        # t2 = t_c - t_offset
        # p2 = ray.point_at_t(t2)
        # n2 = (p2 - self.center) / self.r
        # hit_points.append(HitPoint(ray, self, t1, p1, n1))
        # hit_points.append(HitPoint(ray, self, t2, p2, n2))
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
            hit_points.append(HitPoint(ray, self, t1, p1, n1, self.color))
            hit_points.append(HitPoint(ray, self, t2, p2, n2, self.color))

        return hit_points

class Plane(object):
    def __init__(self, P, normal, color):
        self.P = P
        self.normal = normal
        self.color = color

    def find_hit_point(self, ray):
        hit_points = []
        if ray.dir().dot(self.normal) ==0:
            return hit_points
        t =(self.P - ray.src()).dot(self.normal) /(ray.dir().dot(self.normal))
        if t > 0:
            p = ray.point_at_t(t)
            hit_points.append(HitPoint(ray, self, t, p, self.normal, self.color))
            return hit_points

class Triangle(object):
    def __init__(self, p0, p1, p2, color):
        self.P = p0
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.normal = ((self.p1 - self.p0).cross(self.p2 - self.p0)).norm()
        self.color = color

    def find_hit_point(self, ray):
        hit_points = []
        if ray.dir().dot(self.normal) ==0:
            return hit_points
        t = ( self.P - ray.src()).dot(self.normal) / (ray.dir().dot(self.normal))
        if t < 0:
            return hit_points
        p = ray.point_at_t(t)
        cross00 = (self.p1 - self.p0).cross(p- self.p0)
        cross01 = (self.p2 - self.p1).cross(p - self.p1)
        cross02 = (self.p0 - self.p2).cross(p - self.p2)
        crossList = [cross00,cross01,cross02]
        for c in crossList:
            if self.normal.dot(c.norm()) < 0:
                return hit_points
        hit_points.append(HitPoint(ray, self, t, p, self.normal, self.color))
        return hit_points




def sphereProcess(line):
    index1 = float(line[1])
    index2 = float(line[2])
    index3 = float(line[3])
    r = float(line[4])
    center = vec3(index1,index2, index3)
    if len(colorList) == 0:
        c=vec3(1, 1, 1)
    else:
        c=colorList[-1]
    sphere = Sphere(center, r, c)
    object_list.append(sphere)

def planeProcess(line):
    index1 = float(line[1])
    index2 = float(line[2])
    index3 = float(line[3])
    index4 = float(line[4])
    if index1 != 0:
        p = vec3(-(index4/index1), 0, 0)
    elif index2 != 0:
        p = vec3(0, -(index4/index2), 0)
    else:
        p = vec3(0, 0, -(index4 / index3))
    normal = vec3(index1, index2, index3).norm()
    if len(colorList) == 0:
        c = vec3(1, 1, 1)
    else:
        c = colorList[-1]
    plane = Plane(p, normal, c)
    object_list.append(plane)


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
    if len(colorList)==0:
        light_color.append(vec3(1,1,1))
    else:
        light_color.append(colorList[-1])

def bulbProcess(line):
    x = float(line[1])
    y = float(line[2])
    z = float(line[3])
    lightDir = vec3(x , y, z)
    light_position.append(lightDir)
    if len(colorList)==0:
        light_color.append(vec3(1,1,1))
    else:
        light_color.append(colorList[-1])

def xyzProcess(line):
    x = float(line[1])
    y = float(line[2])
    z = float(line[3])
    pt = vec3(x, y, z)
    return pt

def trifProcess(vertexList,line):
    index1 = int(line[1])
    index2 = int(line[2])
    index3 = int(line[3])
    if index1 > 0:
        index1 -= 1
    if index2 > 0:
        index2 -= 1
    if index3 > 0:
        index3 -= 1
    p0 = vertexList[index1]
    p1 = vertexList[index2]
    p2 = vertexList[index3]
    if len(colorList) == 0:
        c=vec3(1, 1, 1)
    else:
        c=colorList[-1]
    triangle = Triangle(p0, p1, p2, c)
    object_list.append(triangle)

def eyeProcess(line):
    x = float(line[1])
    y = float(line[2])
    z = float(line[3])
    global origin
    origin = vec3(x,y,z)

def forwardProcess(line):
    x = float(line[1])
    y = float(line[2])
    z = float(line[3])
    global f, u, r
    f = vec3(x,y,z)
    f_norm = f.norm()
    r = f_norm.cross(u)
    u =r.cross(f_norm)
    u = (f_norm.cross(u)).cross(f_norm)


def upProcess(line):
    x = float(line[1])
    y = float(line[2])
    z = float(line[3])
    global f , u, r
    u = vec3(x,y,z)
    u = ((f.cross(u)).cross(f)).norm()
    r = f.norm().cross(u)

def aaProcess(line):
    num = int(line[1])
    global num_rays
    num_rays = num


object_list = []
object_color = []
light_position = []
light_color = []
colorList = []
origin = vec3(0,0,0)
f = vec3(0,0,-1)
r = vec3(1,0,0)
u = vec3(0,1,0)
num_rays = 1

vertexList = []

fisheye = False
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
        elif keyword == "eye":
            eyeProcess(l)
        elif keyword == "plane":
            planeProcess(l)
        elif keyword == "bulb":
            bulbProcess(l)
        elif keyword == "forward":
            forwardProcess(l)
        elif keyword == "up":
            upProcess(l)
        elif keyword == "aa":
            aaProcess(l)
        elif keyword == "xyz":
            pt = xyzProcess(l)
            vertexList.append(pt)
        elif keyword == "trif":
            trifProcess(vertexList,l)
        elif keyword == "fisheye":
            fisheye = True

# print(light_color)
# print(light_position)
# print(object_color)
# print(object_list)
def colorTrans(c):
    color = []
    for i in c:
        color.append(round(i * 255))
    r , g, b = color
    c = vec3(r, g, b)
    return c

def colorObject(ray):
    hit_points = []
    for i in range(len(object_list)):
        object = object_list[i]
        points = object.find_hit_point(ray)
        if points != None:
            hit_points.extend(points)
    if hit_points:
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
            n = hit_point.normal
            # print(ray.dir().dot(n))
            if ray.dir().dot(n) > 0:
                n = n.scale(-1)
            # print(l_direction.dot(n))
            o_color = hit_point.color
            color = (o_color * l_color).scale(l_direction.norm().dot(n))
            color = colorTrans(color.toList())
            return color



for i in range(width):
    for j in range(height):
        c = vec3(0,0,0)
        for a in range(num_rays):
            # if fisheye:
            ray = Ray(i , j)
            color = colorObject(ray)
            if color != None:
                c+=color
            else:
                c = color
        if c != None:
            c/= num_rays
            c = c.toIntTuple()
            putpixel((i, j), c)


img.save(filename)