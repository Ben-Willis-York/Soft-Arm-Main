#!/usr/bin/env python

import math

class Vector2(object):
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    '''Add'''
    def __add__(self, other):
        if(type(other) == int) or (type(other) == float):
            return Vector2(self.x+other, self.y+other)
        if(type(other) == Vector2):
            return Vector2(self.x+other.x, self.y+other.y)
        raise TypeError

    '''Sub'''
    def __sub__(self, other):
        if (type(other) == int) or (type(other) == float):
            return Vector2(self.x-other, self.y-other)
        if(type(other) == Vector2):
            return Vector2(self.x-other.x, self.y-other.y)
        raise TypeError

    '''Mult'''
    def __mul__(self, other):
        if(type(other) == int) or (type(other) == float):
            return Vector2(self.x*other, self.y*other)
        if(type(other) == Vector2):
            return Vector2(self.x*other.x, self.y*other.y)
        raise TypeError

    def __div__(self, other):
        if(type(other) == int) or (type(other) == float):
            return Vector2(self.x/other, self.y/other)
        if(type(other) == Vector2):
            return Vector2(self.x/other.x, self.y/other.y)
        raise TypeError

    '''Divide'''
    def __truediv__(self, other):
        if(type(other) == int) or (type(other) == float):
            return Vector2(self.x/other, self.y/other)
        if(type(other) == Vector2):
            return Vector2(self.x/other.x, self.y/other.y)
        raise TypeError

    '''//'''
    def __floordiv__(self, other):
        if (type(other) == int) or (type(other) == float):
            return Vector2(self.x // other, self.y // other)
        if (type(other) == Vector2):
            return Vector2(self.x // other.x, self.y // other.y)
        raise TypeError

    ''' *= '''
    def __imul__(self, other):
        new = self.__mul__(other)
        self.x, self.y = new.x, new.y
        del(new)
        return self

    ''' += '''
    def __iadd__(self, other):
        new = self.__add__(other)
        self.x, self.y = new.x, new.y
        del(new)
        return self

    def __isub__(self, other):
        new = self.__sub__(other)
        self.x, self.y = new.x, new.y
        del(new)
        return self

    def __idiv__(self, other):
        new = self.__truediv__(other)
        self.x, self.y = new.x, new.y
        del(new)
        return self

    def CheckClamp(self):
        if(self.CLAMP == True):
            if(self.x > self.max.x):
                self.x = self.max.x
            if(self.x < self.min.x):
                self.x = self.min.x
            if(self.y > self.max.y):
                self.y = self.max.y
            if (self.y < self.min.y):
                self.y = self.min.y

    def __gt__(self, other):
        if(type(other) == Vector2):
            if(self.x > other.x) and (self.y > other.y):
                return True
            return False
        raise TypeError

    def __lt__(self, other):
        if(type(other) == Vector2):
            if(self.x < other.x) and (self.y < other.y):
                return True
            return False
        raise TypeError

    def __ge__(self, other):
        if(type(other) == Vector2):
            if(self.x >= other.x) and (self.y >= other.y):
                return True
            return False
        raise TypeError

    def __le__(self, other):
        if(type(other) == Vector2):
            if(self.x <= other.x) and (self.y <= other.y):
                return True
            return False
        raise TypeError

    def Mag(self):
        return math.sqrt((self.x**2)+(self.y**2))

    def Normalise(self):
        mag = self.Mag()
        if(mag != 0):
            return (self/self.Mag())
        else:
            return self

    def Clamp(self, Min, Max):
        self.min = Min
        self.max = Max

    def Dot(self, other):
        return (self.x*other.x + self.y*other.y)

    def Det(self, other):
        return(self.x*other.y - self.y*other.x)

    def Angle(self):
        return math.atan2(self.y, self.x)

    def AngleBetween(self, other):
        if(type(other) == Vector2):
            try:
                return math.acos(self.Dot(other))
            except:
                return 0
        else:
            raise TypeError

    def Cross(self, other):
        pass

    '''Print'''
    def __str__(self):
        return str([self.x, self.y])

    def __repr__(self):
        return str([self.x, self.y])

class Vector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    '''Add'''
    def __add__(self, other):
        if (type(other) == int) or (type(other) == float):
            return Vector3(self.x + other, self.y + other, self.z + other)
        if (type(other) == Vector3):
            return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        raise TypeError

    '''Sub'''
    def __sub__(self, other):
        if (type(other) == int) or (type(other) == float):
            return Vector3(self.x - other, self.y - other, self.z - other)
        if (type(other) == Vector3):
            return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
        raise TypeError

    '''Mult'''
    def __mul__(self, other):
        if (type(other) == int) or (type(other) == float):
            return Vector3(self.x * other, self.y * other, self.z * other)
        if (type(other) == Vector3):
            return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)
        raise TypeError

    '''Divide'''
    def __truediv__(self, other):
        if (type(other) == int) or (type(other) == float):
            return Vector3(self.x / other, self.y / other, self.z/other)
        if (type(other) == Vector3):
            return Vector3(self.x / other.x, self.y / other.y, self.z / other.z)

    '''//'''
    def __floordiv__(self, other):
        if (type(other) == int) or (type(other) == float):
            return Vector3(self.x // other, self.y // other, self.z // other)
        if (type(other) == Vector3):
            return Vector3(self.x // other.x, self.y // other.y, self.z//other.z)

    ''' *= '''
    def __imul__(self, other):
        new = self.__mul__(other)
        self.x, self.y, self.z = new.x, new.y, new.z
        del (new)
        return self

    ''' += '''
    def __iadd__(self, other):
        new = self.__add__(other)
        self.x, self.y, self.z = new.x, new.y, new.z
        del (new)
        return self

    def __isub__(self, other):
        new = self.__sub__(other)
        self.x, self.y, self.z = new.x, new.y, new.z
        del (new)
        return self

    def __idiv__(self, other):
        new = self.__truediv__(other)
        self.x, self.y, self.z = new.x, new.y, new.z
        del (new)
        return self

    def CheckClamp(self):
        if (self.CLAMP == True):
            if (self.x > self.max.x):
                self.x = self.max.x
            if (self.x < self.min.x):
                self.x = self.min.x
            if (self.y > self.max.y):
                self.y = self.max.y
            if (self.y < self.min.y):
                self.y = self.min.y

    def __gt__(self, other):
        if (type(other) == Vector3):
            if (self.x > other.x) and (self.y > other.y) and (self.z > other.z):
                return True
            return False
        raise TypeError

    def __lt__(self, other):
        if (type(other) == Vector3):
            if (self.x < other.x) and (self.y < other.y) and (self.z < other.z):
                return True
            return False
        raise TypeError

    def __ge__(self, other):
        if (type(other) == Vector3):
            if (self.x >= other.x) and (self.y >= other.y) and (self.z >= other.z):
                return True
            return False
        raise TypeError

    def __le__(self, other):
        if (type(other) == Vector3):
            if (self.x <= other.x) and (self.y <= other.y) and (self.z <= other.z):
                return True
            return False
        raise TypeError

    def Mag(self):
        return math.sqrt((self.x*self.x) + (self.y*self.y) + (self.z*self.z))

    def Normalise(self):
        mag = self.Mag()
        if (mag != 0):
            return (self / self.Mag())
        else:
            return self

    def Clamp(self, Min, Max):
        self.min = Min
        self.max = Max

    def Dot(self, other):
        if(type(other) == Vector3):
            return (self.x * other.x + self.y * other.y + self.z * other.z)
        raise TypeError


        '''Print'''
    def __str__(self):
        return str([self.x, self.y, self.z])

    def __repr__(self):
        return str([self.x, self.y, self.z])


class BBox:
    def __init__(self, p1, p2):
       self.top = p1
       self.bottom = p2

    def PointInside(self, p):
        if (p >= self.bottom) and (p <= self.top):
            return True
        else:
            return False

class Collision:
    def __init__(self, p1, p2):
        self.p1, self.p2 = p1, p2
        between = p1.pos - p2.pos;
        self.N = between.Normalise()
        self.W = 1 - (between.Mag()/(p1.radius+p2.radius))
