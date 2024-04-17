import math
import copy
import threading
import time
from threading import Thread

alpha = 50

class Point:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
        self.matched = False

    def __list__(self):
        return [self.lat, self.lon]

    def equals(self, point):
        return -0.000000001<=self.lat - point.lat<=0.000000001 and -0.000000001<=self.lon == point.lon<=0.000000001

def interpolation(point1, point2, dis):
    if calcDist(point1, point2) < dis:
        return [point1]
    else:
        segNum = math.ceil(calcDist(point1, point2) / dis)
        pointList = []
        dx = (point2.lat - point1.lat) / segNum
        dy = (point2.lon - point1.lon) / segNum
        for i in range(0, segNum):
            pointList.append(Point(point1.lat + i * dx, point1.lon + i * dy))
        return pointList

def calcDist(point1, point2):
    normed_dist = calcNormalizedDist(point1, point2)
    return 1.2742E7 * math.asin(math.sqrt(normed_dist))

def calcNormalizedDist(point1, point2):
    sin_delta_lat = math.sin(math.radians(point2.lat - point1.lat) / 2.0)
    sin_delta_lon = math.sin(math.radians(point2.lon - point1.lon) / 2.0)
    return sin_delta_lat * sin_delta_lat + sin_delta_lon * sin_delta_lon * math.cos(
        math.radians(point1.lat)) * math.cos(
        math.radians(point2.lat))

def calcErr(matchPath, realPath):
    if len(matchPath) == 0 or len(realPath) == 0:
        return 1
    mp = []
    rp = []
    for i in range(0, len(matchPath) - 1):
        mp.extend(interpolation(matchPath[i], matchPath[i + 1], 20))
    mp.append(matchPath[len(matchPath) - 1])
    for i in range(0, len(realPath) - 1):
        rp.extend(interpolation(realPath[i], realPath[i + 1], 20))
    rp.append(realPath[len(realPath) - 1])
    time1 = time.time()
    i = 0
    for p1 in mp:
        j = i
        while j < len(rp):
            if calcDist(p1, rp[j]) < alpha:
                p1.matched = True
                i = max(0, j - 5)
                break
            j += 1

    i = 0
    for p1 in rp:
        j = i
        while j < len(mp):
            if calcDist(p1, mp[j]) < alpha:
                p1.matched = True
                i = max(0, j - 5)
                break
            j += 1

    unMatchedNum = 0
    for p in mp:
        if not p.matched:
            unMatchedNum += 1
    for p in rp:
        if not p.matched:
            unMatchedNum += 1
    return unMatchedNum / len(rp)

def calcErrs(matchPaths,realPaths):
    lock1 = threading.Lock()
    activeThreadNum = 0

    class MyThread(Thread):
        def __init__(self, id, func, arg1, arg2):
            Thread.__init__(self)
            self.func = func
            self.id = id
            self.result = None
            self.arg1 = arg1
            self.arg2 = arg2

        def run(self):
            nonlocal activeThreadNum
            with lock1:
                activeThreadNum += 1
            self.result = self.func(self.arg1, self.arg2)
            with lock1:
                activeThreadNum -= 1

        def getResult(self):
            return self.result


    errs = [-1 for i in range(0,len(matchPaths))]
    tasks = []

    for i in range(0,len(matchPaths)):
        task = MyThread(i,calcErr,matchPaths[i],realPaths[i])
        tasks.append(task)

    for task in tasks:
        while activeThreadNum > 32:
            time.sleep(0.1)
        task.start()

    for task in tasks:
        task.join()
        err = task.getResult()
        errs[task.id] = err
    return errs
