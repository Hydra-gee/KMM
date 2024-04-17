import math
import random
import time
from datetime import datetime

from gymnasium.envs.registration import register
import gymnasium
import numpy as np
import socket
import requests
import urllib3
import json
from threading import Thread
import pandas as pd

from calcErr import Point, calcErr
from utils import parse_xml
gpxDir = r"../trajectories/"
register(
    id="mapmatching-v0",
    entry_point="mmEnv:MMEnv",
)

lambda_ = 100

def sendRequest(index):
    print("index:",index)
    with open(gpxDir+r'{}.gpx'.format(index), 'rb') as f:
        r = requests.post(url, data=f, headers={'Content-Type': 'application/gpx+xml'})
        j = json.loads(r.text)
        return index,j

class MyThread(Thread):
    def __init__(self, func, arg):
        Thread.__init__(self)
        self.func = func
        self.result = None
        self.arg = arg

    def run(self):
        self.result = self.func(self.arg)

    def getResult(self):
        return self.result


def allowed_gai_family():
    return socket.AF_INET


urllib3.util.connection.allowed_gai_family = allowed_gai_family
kp_dis_threshold = 50
kp_connum_threshold = 3
algorithm = 2
pathCalcAlgo = 0
pathCacheType = 1
totalDataNum = 2700
weightType = 0 # 0:distance 1:time

url = 'http://localhost:8999/match?profile=car&type=json&gps_accuracy=20&keypoint_num=999&kp_dis_threshold={}&kp_connum_threshold={}&algorithm={}&useVelocity=true&velocityThreshold=35&path_calc_algo={}&path_cache_type={}&weight_type={}'.format(kp_dis_threshold,kp_connum_threshold,algorithm,pathCalcAlgo,pathCacheType,weightType)

class MMEnv(gymnasium.Env):
    metadata = {}

    def __init__(self, render_mode=None):
        # Observations are dictionaries with the agent's and the target's location.
        self.realPath = None
        self.action_space = gymnasium.spaces.Discrete(2)
        self.observation_space = gymnasium.spaces.Box(
            -np.inf, np.inf, shape=(7,), dtype=np.float64
        )
        self.trace_idx = None
        self.serverSocket = None
        self.clientSocket = None
        self.task = None
        self.lasttime = 0
        self.ts = 0
        self.starttime = 0
        self.kpNum = 0
        self.logCache = pd.DataFrame(columns=["id","difference","time","kpNum"])
        self.ruleResult = pd.read_csv("../result/statistics_shanghai_KMM.csv")
        self.buildServer()

    def buildServer(self):
        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = "localhost"
        port = 7878
        self.serverSocket.bind((host, port))
        self.serverSocket.listen(1)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.kpNum = 0
        if options and 'trace_idx' in options:
            self.trace_idx = options['trace_idx']
        else:
            self.trace_idx = random.randint(int(totalDataNum*0.3), totalDataNum-1)
        self.task = MyThread(sendRequest, self.trace_idx)
        self.realPath = []
        filePath = gpxDir + str(self.trace_idx) + ".gpx"
        _, x = parse_xml(filePath)
        for i in range(0, len(x)):
            self.realPath.append(Point(x.iloc[i]['lat'], x.iloc[i]['lon']))
        self.task.start()
        self.clientSocket, addr = self.serverSocket.accept()
        msg = self.clientSocket.recv(1024)
        observation = json.loads(msg.decode("utf-8"))
        info = {}
        self.starttime = time.time()
        return np.array([observation['state']]), info

    def step(self, action):
        self.ts += 1
        self.clientSocket.send((str(action)+"\n").encode('utf-8'))
        msg = self.clientSocket.recv(2048)

        result = json.loads(msg.decode("utf-8"))
        result['reward'] = 0
        if action == 1:
            self.kpNum += 1
            if self.kpNum>self.ruleResult.iloc[self.trace_idx,3]:
                result['reward'] -= 1
        if result['done']:
            print("done")
            self.task.join()
            fileIdx,jsonResult = self.task.getResult()
            matchPath = []
            for i in range(0, len(jsonResult['latitudes'])):
                matchPath.append(Point(jsonResult['latitudes'][i], jsonResult['longitudes'][i]))
            err = calcErr(matchPath, self.realPath)
            new_row = pd.Series([str(fileIdx), err,time.time()-self.starttime,self.kpNum], index=self.logCache.columns)
            self.logCache.loc[len(self.logCache)] = new_row.values
            ruleErr = min(0.5,self.ruleResult.iloc[fileIdx,1])
            rlErr = min(0.5,err)
            result['reward'] += -lambda_ * math.log(1 + rlErr - ruleErr)
            result['err'] = err
        info = {}
        return np.array([result['state']]), result['reward'], result['done'], False, info

    def save(self):
        print("save")
        current_time = datetime.now()
        self.logCache.to_csv("rlLog{:02d}-{:02d}-{:02d}.csv".format(current_time.day, current_time.hour, current_time.minute),index=False)

