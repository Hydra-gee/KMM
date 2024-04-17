import os

import requests
import json
import urllib3
from calcErr import Point, calcErrs
from utils import allowed_gai_family, parse_xml

urllib3.util.connection.allowed_gai_family = allowed_gai_family

kp_dis_threshold = 50 #deviation distance threshold (γ)
kp_connum_threshold = 3 #deviant point number threshold (θ)
algorithm = 3 # 0:SP, 1:KMM(rule_based), 3:ST-MM
pathCalcAlgo = 0  # 0：CH, 1:Dijkstra
pathCacheType = 1  # 0:no cache 1:cache
weightType = 1 # 0:distance 1:time
city = "shanghai"

if algorithm == 0:
    algorithmName = "SP"
elif algorithm == 1:
    algorithmName = "KMM"
elif algorithm == 3:
    algorithmName = "ST-MM"
else:
    raise Exception("algorithm err")

url = 'http://localhost:8999/match?profile=car&type=json&gps_accuracy=20&keypoint_num=999&kp_dis_threshold={}&kp_connum_threshold={}&algorithm={}&useVelocity=true&velocityThreshold=60&path_calc_algo={}&path_cache_type={}&weight_type={}'.format(
    kp_dis_threshold, kp_connum_threshold, algorithm, pathCalcAlgo, pathCacheType,weightType)
matchPaths = []
groundPaths = []
times = []
kpNums = []

gpxDir = ".\\trajectories\\"
outputDir = ".\\result\statistics_{}_{}.csv".format(city, algorithmName)

num = len(os.listdir(gpxDir))

# for fileName in os.listdir(gpxDir):
for id in range(0, num):
    filePath = gpxDir + str(id) + ".gpx"
    with open(filePath, 'rb') as f:
        r = requests.post(url, data=f, headers={'Content-Type': 'application/gpx+xml'})
        if r.status_code == 200:
            j = json.loads(r.text)
            _, x = parse_xml(filePath)
            matchPath = []
            realPath = []
            for i in range(0, len(j['latitudes'])):
                matchPath.append(Point(j['latitudes'][i], j['longitudes'][i]))
            for i in range(0, len(x)):
                realPath.append(Point(x.iloc[i]['lat'], x.iloc[i]['lon']))
            matchPaths.append(matchPath)
            groundPaths.append(realPath)
            times.append(j['duration'])
            if algorithm == 1:
                kpNums.append(j['keypointNum'])
            print("processed " + str(id))
print("calculating errs......")
errs = calcErrs(matchPaths, groundPaths)
with open(outputDir, "w") as f:
    if algorithm == 1:
        f.write("id,err,time,kpNum\n")
        for i in range(0, num):
            f.write(str(i) + "," + str(errs[i]) + "," + str(times[i]) + "," + str(kpNums[i]) + "\n")
    else:
        f.write("id,err,time\n")
        for i in range(0, num):
            f.write(str(i) + "," + str(errs[i]) + "," + str(times[i]) + "\n")

print("done")
print("output to " + outputDir)