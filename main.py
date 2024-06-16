import math
import os
import sys
import argparse

from discopygal.solvers import *
from modules.rrt_solver import RRTSolver
from modules.prm_solver import CustomPRM

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

###################
# parsing methods #
###################

def initParser():
    """
        initialize arguments parser
    """
    parser = argparse.ArgumentParser(prog=__name__, description='')
    
    parser.add_argument('-scene', help="Path to the scene to run", type=str,required=True)
    parser.add_argument('--with-gui',help="boolean for gui option", type=int, default=0)
    parser.add_argument('--num-landmarks', help="Number of landmarks to connect", type=int, default=1000)
    parser.add_argument('--k', help="Number of neighbors to connect with in knn", type=int, default=15)
    parser.add_argument('--solver', help="Choose which solver to use", type=str, default="prm")
    parser.add_argument('--eta', help="Eta const value to use with rrt solver", type=float, default=math.inf)
    parser.add_argument('--star', help="Choose if you want to run rrt* algorithm", type=int, default=0)
    parser.add_argument('--output', help="Path of the output file", type=str, default="output.txt")
    parser.add_argument('-v', '--verbose', help='enable debug mode',action='store_true')
    return parser

def parseJson(sceneName: str) -> tuple:
    """
        extract the obstacles, robots and metadata description from the scene .json format
        @param sceneName: a string type object for the .json file scene description
    """
    import json
    with open(sceneName, 'r') as f:
        data = json.load(f)
        obstaclesData = [ObstaclePolygon.from_dict(d=_obs) for _obs in data['obstacles']]
        robotData = [RobotDisc.from_dict(d=_rod) for _rod in data['robots']] 
        metadata = data['metadata']

        return obstaclesData, robotData, metadata

def validateInput(args):
    assert args.num_landmarks > 0, "num_landmarks must be greater than 0"
    assert args.k > 0, "k must be greater than 0"
    assert args.eta > 0, "eta must be greater than 0"
    

def main():
    parser = initParser()
    args = parser.parse_args()
    validateInput(args)
    
    withGui = args.with_gui
    sceneName = args.scene
    numLandmarks = args.num_landmarks
    k = args.k
    solver = args.solver
    eta = args.eta
    is_star = args.star
    output = args.output
    verbose = args.verbose
    if verbose:
        print("\DEBUG mode")
    
    # configure scene and solver
    obstacles, robot, metadata = parseJson(sceneName=sceneName)
    scene = Scene(obstacles=obstacles, robots=robot, metadata=metadata)
    if solver == "prm":
        CustomPRM.PRMStart(scene=scene, numLandmarks=numLandmarks, k=k, withGui=withGui,output=output,verbose=verbose)
    elif solver == "rrt":
        RRTSolver.RRTStart(scene=scene, numLandmarks=numLandmarks, eta=eta, withGui=withGui,is_star=is_star,output=output,verbose=verbose)
    else:
        raise ValueError(f"Invalid solver value: '{solver}', must be 'prm' or 'rrt'")

if __name__ == "__main__":
    rc = 1
    try:
        main()
        rc = 0
    except Exception as e:
        print('Error: %s' % e, file=sys.stderr)
    sys.exit(rc)