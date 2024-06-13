

import json
import math
import os
import random
import subprocess
import threading
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--output', help='Name of outputfile', default="report")
parser.add_argument('--laps', help='Laps number per scene to test', default=5,type=int)
args = parser.parse_args()

report_name = args.output
LAPS = args.laps
K = 15
OUTPUT_FOLDER = 'experiments'

scenesMap = {
    "3 Discs Custom": 'inputs/3_discs_custom.json',
    "4 Robots": 'inputs/4_robots.json',
    "Hard Test 2 Robots": 'inputs/2_robots_hard_test.json',
    "5 Robots": 'inputs/5_robots.json',
    "5 Robots Hard": 'inputs/5_robots_hard.json',
    "3 Robots Hard Maze": 'inputs/3_robots_hard_maze.json',
    "3 Discs custom #2": 'inputs/3_discs_custom_2.json'
}


def runExperiment(scenePath: str, solver: str, numLandmarks: int,eta=math.inf,star=0,K=K):

    score=0
    args = ["python3", "main.py",
            "-scene", scenePath,
            "--with-gui", "0",
            "--num-landmarks", str(numLandmarks),
            "--k", str(K),
            "--solver", solver,
            "--star", str(star)]
    if eta != math.inf:
        args.extend(["--eta", str(eta)])
        
    seconds = random.randint(2,5)
    time.sleep(seconds)
    start = time.time()
    subprocess.run(args)
    end = time.time()
    
    with open('output.txt', 'r') as output:
        lines = output.readlines()
        SUCCEED = 'No path found' not in lines[0]
        if SUCCEED:
            _, score = lines[-1].split()
    total = end - start
    return total, float(score), SUCCEED


def startExperiment():
    
    def runSceneNLandmarks(sceneName,scenePath,n):
        
        global K
        print(f'{n} landmarks')
        totalTimePRM, totalTimeRRT, totalTimeRRTStar = 0, 0, 0
        succesPRM, successRRT, successRRTStar = 0, 0, 0
        distancePRM, distanceRRT, distanceRRTStar = [], [], []
        eta = 2 if sceneName == "3 Robots Hard Maze" else math.inf
        if sceneName == "5 Robots Hard":
            eta = 5
            K = 20
            
        for _ in range(LAPS):
            print('running PRM...')
            tPRM, scorePRM, resPRM = runExperiment(scenePath=scenePath, solver='prm', numLandmarks=n,K=K)
            print('running RRT...')
            tRRT, scoreRRT, resRRT = runExperiment(scenePath=scenePath, solver='rrt', numLandmarks=n,eta=eta)
            print('running RRT*...')
            tRRTStar, scoreRRTStar, resRRTStar = runExperiment(scenePath=scenePath, solver='rrt', numLandmarks=n,eta=eta,star=1)
            
            totalTimePRM += tPRM
            totalTimeRRT += tRRT
            totalTimeRRTStar += tRRTStar
            succesPRM += 1 if resPRM else 0
            successRRT += 1 if resRRT else 0
            successRRTStar += 1 if resRRTStar else 0
            if (resPRM):
                distancePRM.append(scorePRM)
            if (resRRT):
                distanceRRT.append(scoreRRT)
            if (resRRTStar):
                distanceRRTStar.append(scoreRRTStar)


        results[sceneName][f"{n} Landmarks"] = {
            "PRM": {
                "avg time": totalTimePRM / LAPS,
                "success rate": succesPRM / LAPS,
                "avg distance": sum(distancePRM) / max(1, len(distancePRM))
            },
            "RRT": {
                "avg time": totalTimeRRT / LAPS,
                "success rate": successRRT / LAPS,
                "avg distance": sum(distanceRRT) / max(1, len(distanceRRT)) 
            },
            "RRT*": {
                "avg time": totalTimeRRTStar / LAPS,
                "success rate": successRRTStar / LAPS,
                "avg distance": sum(distanceRRTStar) / max(1, len(distanceRRTStar))
            }
        }
    
    def runScene(sceneName,scenePath):
        print(f'\nRunning {sceneName}')
        
        numLandmarks = [2000, 3500, 5000]
        if sceneName == "3 Robots Hard Maze":
            numLandmarks = [12500, 15000, 17500]
        if sceneName == "5 Robots Hard":
            numLandmarks = [15000, 17500, 20000]
        
        iter_threader = [threading.Thread(target=runSceneNLandmarks, args=[sceneName,scenePath,n]) for n in numLandmarks]
        for th in iter_threader: #runs the task in threads
            th.start()
        for th in iter_threader: #sync the threads before moving to the next iteration or exit the loop
            th.join()
            

    results = {}
    for scene in scenesMap.keys():
        results[scene] = dict()
    print('***********Start Experiment***********')
     
    iter_threader = [threading.Thread(target=runScene, args=[sceneName,scenePath]) for sceneName, scenePath in scenesMap.items()]
    for t in iter_threader: #runs the task in threads
        t.start()
    for t in iter_threader: #sync the threads before moving to the next iteration or exit the loop
        t.join()
        
    # Check if the folder exists, if not, create it
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)

    # File path
    file_path = os.path.join(OUTPUT_FOLDER, f"{report_name}.json")

    # Write the dictionary to a JSON file
    with open(file_path, 'w') as json_file:
        json.dump(results, json_file, indent=4)
        

if __name__ == "__main__":
    startExperiment()