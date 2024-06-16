import math
import os
import sys
import networkx as nx

from discopygal.solvers import prm, verify_paths, Solver
from discopygal.solvers import *
from discopygal_tools.solver_viewer import start_gui as startGui
from discopygal.bindings import *
from discopygal.geometry_utils import collision_detection

PRM = prm.PRM
verifyPaths = verify_paths.verify_paths

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


###################
#     solver      #
###################

class CustomPRM(PRM):
    def __init__(self, num_landmarks, k, bounding_margin_width_factor=Solver.Solver.DEFAULT_BOUNDS_MARGIN_FACTOR, nearest_neighbors=None, metric=None, sampler=None):
        super().__init__(num_landmarks, k, bounding_margin_width_factor)


    def load_scene(self, scene: Scene,verbose: bool):
        """
        Loads the specified scene into the environment

        @param scene: a Scene type object representing the environment to be loaded

        The method creates the scene by:
        1. Building the graph by adding valid points + source and target to it.
        2. Connecting all points to their k nearest neighbors.
        """
        self.scene = scene
        self._bounding_box = self.calc_bounding_box()
        self.sampler.set_scene(scene, self._bounding_box)

        # Build collision detection for each robot
        for robot in scene.robots:
            self.collision_detection[robot] = collision_detection.ObjectCollisionDetection(scene.obstacles, robot)

        ################
        # Build the PRM
        ################
        self.roadmap = nx.Graph()

        # Add start & end points
        self.start = conversions.Point_2_list_to_Point_d([robot.start for robot in scene.robots])
        self.end = conversions.Point_2_list_to_Point_d([robot.end for robot in scene.robots])
        self.roadmap.add_node(self.start)
        self.roadmap.add_node(self.end)
        
        # Add valid points
        print(f"\nRUNNING PRM - {self.num_landmarks} landmarks\n")
        for _ in range(self.num_landmarks):
            p_rand = self.sample_free()
            self.roadmap.add_node(p_rand)

        self.nearest_neighbors.fit(list(self.roadmap.nodes))

        # Connect all points to their k nearest neighbors
        for cnt, point in enumerate(self.roadmap.nodes):
            neighbors = self.nearest_neighbors.k_nearest(point, self.k+1)
            for neighbor in neighbors:
                if self.collision_free(neighbor, point):
                    self.roadmap.add_edge(point, neighbor, weight=CustomPRM.customDist(point, neighbor))

            if verbose and cnt % 100 == 0:
                print('connected', cnt, 'landmarks to their nearest neighbors')
    
    def solve(self):
        """
        Based on the start and end locations of each robot, solve the scene
        (i.e. return paths for all the robots)

        @return: path collection of motion planning
        @rtype: :class:`~discopygal.solvers.PathCollection`
        """
        if not nx.algorithms.has_path(self.roadmap, self.start, self.end):
            return PathCollection()

        # Convert from a sequence of Point_d points to PathCollection
        print(f'\nPRM found valid path:\n')
        tensor_path = nx.algorithms.shortest_path(self.roadmap, self.start, self.end, weight='weight')
        path_collection = PathCollection()
        for i, robot in enumerate(self.scene.robots):
            points = []
            for point in tensor_path:
                if i==0:
                    print(point)
                points.append(PathPoint(Point_2(point[2*i], point[2*i+1])))
            path = Path(points)
            path_collection.add_robot_path(robot, path)

        return path_collection

    @staticmethod
    def customDist(p, q):
        """
        Calculates the Euclidean distance between two points

        @param p: a Point_d type object representing the first point
        @param q: a Point_d type object representing the second point

        The function calculates and returns the Euclidean distance between the two points p and q.
        """
        pList, qList = conversions.Point_d_to_Point_2_list(p), conversions.Point_d_to_Point_2_list(q)
        distance=0
        for i in range(len(pList)):
            distance += math.pow(pList[i].x().to_double() - qList[i].x().to_double(), 2) +\
                         math.pow(pList[i].y().to_double() - qList[i].y().to_double(), 2)
        return distance**0.5

    @staticmethod
    def PRMStart(scene: Scene, numLandmarks: int, k: int, withGui: bool,output="output.txt",verbose=False):
        """
        Runs the PRM algorithm for path planning in a given scene

        @param scene: a Scene type object representing the environment where the PRM algorithm will be executed
        @param numLandmarks: integer value for the number of landmarks to pick
        @param k: integer value for the number of neighbors to connect with the nearest neighbor graph
        @param withGui: boolean flag for displaying GUI during the execution of the algorithm

        The function initializes the scene, calls the solver function to run the PRM algorithm,
        displays the GUI if withGui is True, and writes the result to an output file.
        """
        basicPRMSolver = CustomPRM(num_landmarks=numLandmarks, k=k)
        if withGui:
                # Start solver_viewer with the scene and solver objects (the scene isn't solved yet)
                startGui(scene=scene, solver=basicPRMSolver)
                return
        basicPRMSolver.load_scene(scene=scene,verbose)
        path = basicPRMSolver.solve()
        result, _ = verifyPaths(scene=scene, paths=path)
        with open(output, 'w') as f:
            if not result:
                print(f"\nPRM didn\'t find valid path")
                f.write(f"No path found\n")
            else:
                robots_points = [path.paths[robot].points for robot in scene.robots]
                path_list=[]
                for points in zip(*robots_points):
                    node=[]
                    node_str = ""
                    for point in points:
                        node.append(Point_2(point.location[0],point.location[1])) 
                        node_str += f"{point.location[0]} {point.location[1]} "
                    f.write(f"{node_str}\n")
                    path_list.append(node)
                cost = 0
                for i in range(len(path_list)-1):
                    cost+= CustomPRM.customDist(conversions.Point_2_list_to_Point_d(path_list[i]),conversions.Point_2_list_to_Point_d(path_list[i+1]))
                print(F"\nCOST {cost}")
                f.write(f"\nCOST {cost}")
                f.close()