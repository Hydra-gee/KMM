## source code for KMM

### Structure:
the "Algorithm" dictionary contains the core algorithm of KMM, implemented in Java, based on the open source code of Graphhopper https://github.com/graphhopper/graphhopper.

the "Dataprocess" dictionary contains the test scripts and the RL environment.

the "Dataprocess/trajectories" dictionary contains a subset of our test trajectories in shanghai dataset.

### Usage:

1. config the maven project in "Algorithm" Dictionary, and run the KMMApplication to start the server.

2. in the "Dataprocess" Dictionary, run the "rulebased_batch.py" matching all the data in the "trajectories" dictionary, and generate a statistic of the accuracy and time in the "result" dictionary.

3. "rulebased.ipynb" matches a specified path and displays it on Google Maps.

<img src=".\pic\rule_based.png" width=300>

4. the "Dataprocess/RL" dictionary contains the environment and script to run the RL-based method. Before running RL training and testing, it is necessary to place the trajectory data in the trajectories folder and number them starting from 0. Then, run rule-based_batch.py to generate the benchmark. The model will use the first 20% of the data as the test set and the last 70% of the data as the training set. To ensure effectiveness, at least 1000 trajectory data should be included.

### Configuration:
1. Your map file (.osm / .osm.pbf) should be placed in the "Maps" dictionary. The path in "KMMApplication" should also be modified:
<img src=".\pic\2.png" >

2. the "totalDataNum" in "mmEnv.py" and "rltest.py" should be the same as the number of data in "trajectories" folder.
<img src=".\pic\3.png" >
<img src=".\pic\4.png" >

3. a illustration of the parameters:
<img src=".\pic\5.png" >





