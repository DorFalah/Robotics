import random
import math
import numpy as np
import networkx as nx

from discopygal_tools.solver_viewer import start_gui
from discopygal.solvers import Scene

from discopygal.solvers import Solver, Scene, samplers, PathPoint, PathCollection, Path
from discopygal.bindings import *
from discopygal.solvers import verify_paths, Solver
from discopygal.solvers import *
from discopygal.bindings import *
from discopygal.geometry_utils import collision_detection

verifyPaths = verify_paths.verify_paths

RRTSTAR_FULL_ITERATION_LIMIT = 1000
RRTSTAR_MODULU_FULL_ITERATION = 100
RRT_FULL_NEIGHBOR_SEARCH_LIMIT = 2000

class RRTSolver(Solver.Solver):
    def __init__(self, numLandmarks: int, eta: float,is_star: int):
        super().__init__(bounding_margin_width_factor=Solver.Solver.DEFAULT_BOUNDS_MARGIN_FACTOR)
        self.collision_detection = {}
        self.numLandmarks = numLandmarks
        self.eta = eta
        self.is_star = is_star
        self.cost = 0
    

    def load_scene(self, scene: Scene):
        """
        Loads the specified scene into the environment

        @param scene: a Scene type object representing the environment to be loaded

        The method performs the following steps:
        1. Sets the scene.
        2. Initializes collision detection for each robot in the scene with respect to the obstacles.
        3. Initializes a uniform sampler and sets the scene and bounding box for the sampler.
        4. Sets the dimension of the configuration space based on the number of robots.
        """
        self.scene = scene
        for robot in scene.robots:
            self.collision_detection[robot] = collision_detection.ObjectCollisionDetection(scene.obstacles, robot)
        self.sampler = samplers.Sampler_Uniform()
        self.sampler.set_scene(scene, self._bounding_box)
        self.dim = 2*len(self.scene.robots)
    

    def solve(self):
        """
        Executes the path planning algorithm to find a solution for the given scene

        The method performs the following steps:
        1. Initializes the start and end points for each robot in the scene.
        2. Determines the type of algorithm to use (either RRT or RRT*).
        3. Builds the graph representing the environment.
        4. Connects the target to the graph if it's not already connected.
        5. Checks if a path exists between the source and target points.
        6. If a path exists, calculates the shortest path using Dijkstra's algorithm.
        7. Initializes the paths, calculates the cost and prints them.
        9. Returns a collection of paths if a solution is found, otherwise returns None.

        @return: A path collection of paths if a solution is found, otherwise returns None.
        @rtype: :class:`~discopygal.solvers.PathCollection`
        """
        self.start = conversions.Point_2_list_to_Point_d([robot.start for robot in self.scene.robots])
        self.end = conversions.Point_2_list_to_Point_d([robot.end for robot in self.scene.robots])
        target = self.end
        source = self.start
        shortestPath = None
        paths = dict()
        algorithm = "RRT*" if self.is_star else "RRT"
        
        print(f"\nRUNNING '{algorithm}' - {self.numLandmarks} landmarks with {self.eta} eta\n")

        self.buildGraph()
        if target not in self.graph.nodes():
            print(f"Connecting target to the graph")
            if not self.connectTargetToGraph(target=target):
                print(f"Failed")
                return None
        if not nx.has_path(G=self.graph, source=source, target=target):
            return None
        print(f"Success\n")
        shortestPath = nx.dijkstra_path(G=self.graph, source=source, target=target)
        self.printAndInitPaths(paths,shortestPath)
        self.calcAndPrintCost(shortestPath)
            
        return PathCollection(paths)
    
    
    def printAndInitPaths(self,paths,shortestPath):
        """
        Initializes paths for each robot and prints the shortest path

        @param paths: a dictionary where keys are robots and values are lists to store the path points
        @param shortestPath: an ArrayLike representing the shortest path found by the algorithm

        The method performs the following steps:
        1. Initializes an empty path list for each robot.
        2. Prints each vertex in the shortest path.
        3. Converts each vertex in the shortest path to a PathPoint object and appends it to the corresponding robot's path.
        4. Converts the list of PathPoint objects for each robot into a Path object.
        """
        for robot in self.scene.robots:
            paths[robot] = []
        print("Path:")
        for i, v in enumerate(shortestPath):
            print(v)
            for j,robot in enumerate(self.scene.robots):
                paths[robot].append(PathPoint(Point_2(FT(v[2*j]), FT(v[2*j+1]))))
        for robot in self.scene.robots:
            path = paths[robot]
            paths[robot] = Path(path)
    
    def calcAndPrintCost(self,shortestPath):
        """
        Calculates and prints the cost of the shortest path

        @param shortestPath: an ArrayLike representing the shortest path found by the algorithm

        The method performs the following steps:
        1. Retrieves the second-to-last node in the shortest path.
        2. Calculates the total cost by adding the cost of the second-to-last node
           and the distance between the last two nodes in the path.
        3. Prints the total cost.
        """
        before_last_node = shortestPath[-2]
        self.cost = self.graph.nodes[before_last_node]['cost'] + RRTSolver.dist(shortestPath[-2],shortestPath[-1])
        print(f"\nCOST {self.cost}")
        

    def sampleValidPoint(self):
        """
        Samples a valid point for each robot in the scene.

        The method performs the following steps:
        1. Iterates through each robot in the scene.
        2. Samples a point using the uniform sampler.
        3. Checks if the sampled point is valid using the collision detection object for the current robot.
        4. If the sampled point is not valid, continues sampling until a valid point is found.
        5. Adds the valid sampled point to a list.
        6. Returns a Point_d object after converting from Point_2_list .

        @return: A Point_d object containing the valid sampled points for each robot.
        """
        p_rand = []
        for robot in self.scene.robots:
            sample = self.sampler.sample()
            while not self.collision_detection[robot].is_point_valid(sample):
                sample = self.sampler.sample()
            p_rand.append(sample)
        return conversions.Point_2_list_to_Point_d(p_rand)
    
    
    def isEdgeValidPerRobot(self,node_in_graph,target_node):
        """
        Checks if an edge between two nodes is valid for each robot in the scene.

        @param node_in_graph: A Point_d representing a node currently in the graph.
        @param target_node: A Point_d representing the target node to connect to the graph.

        The method performs the following steps:
        1. Iterates through each robot in the scene.
        2. Constructs an edge between the current node and the target node.
        3. Checks if the edge is valid for the current robot using the collision detection object.
        4. Returns True if the edge is valid for all robots, False otherwise

        @return: True if the edge is valid for all robots, otherwise False.
        """
        x_point = conversions.Point_d_to_Point_2_list(node_in_graph)
        for i, robot  in enumerate(self.scene.robots):
            point_i = Point_2(FT(target_node[2*i]), FT(target_node[2*i+1]))
            edge= Segment_2(point_i, x_point[i])
            if not self.collision_detection[robot].is_edge_valid(edge=edge):
                return False
        return True
    
    def rrtStarStep(self,ret,nearestNeighbor):
        """
        Performs an iteration step of the RRT* algorithm to improve the graph connectivity.

        @param ret: A Point_d representing the newly added point to the graph.
        @param nearestNeighbor: A tuple representing the nearest neighbor to the newly added point.

        The method performs the following steps:
        1. Retrieves XNEAR of the newly added point.
        2. Initializes the minimum cost `c_min` with the cost of the newly added point.
        3. Iterates through each nearest node and checks if the edge
           between it and the new point is collision-free for all robots.
        4. If the edge is valid, calculates the cost to reach the new point through the current nearest node.
        5. Updates the minimum cost if needed.
        6. Adds edges between the nearest node and the new point.
        7. Iterates again through the nearest nodes to check for further improvements in each node cost.
        8. If an improvement is found, updates the cost of the nearest node and adds corresponding edges.
        """
        X_NEAR = self.getXNear(ret)
        c_min = self.graph.nodes[ret]['cost']
        for x in X_NEAR:
            if self.isEdgeValidPerRobot(x,ret):
                if self.checkEdge(x,ret):
                    cost = self.graph.nodes[x]['cost'] + RRTSolver.dist(x, ret)
                    if cost < c_min:
                        c_min = cost
                        nearestNeighbor = x
        self.graph.nodes[ret]['cost'] = c_min
        self.graph.add_edge(u_of_edge=nearestNeighbor, v_of_edge=ret, weight=RRTSolver.dist(p=nearestNeighbor, q=ret))
        self.graph.add_edge(u_of_edge=ret, v_of_edge=nearestNeighbor, weight=RRTSolver.dist(p=nearestNeighbor, q=ret))
        for x in X_NEAR:
            if self.isEdgeValidPerRobot(x,ret):
                if self.checkEdge(x,ret):
                    improve_dist = RRTSolver.dist(x,ret)
                    if self.graph.nodes[ret]['cost'] + improve_dist < self.graph.nodes[x]['cost']:
                        self.graph.nodes[x]['cost'] = self.graph.nodes[ret]['cost'] + improve_dist
                        self.graph.add_edge(u_of_edge=x, v_of_edge=ret, weight=improve_dist)
                        self.graph.add_edge(u_of_edge=ret, v_of_edge=x, weight=improve_dist)
        
            
    def connectTargetToGraph(self, target: tuple):
        """
        Connects the target point to the existing graph while ensuring collision-free edges.

        @param target: A Point_d representing the target point to connect to the graph.

        The method performs the following steps:
        1. Iterates through each node in the graph.
        2. Calculates the distance between each node and the target point.
        3. Checks if an edge between the current node and the target point is collision-free for all robots.
        4. If the edge is collision-free, adds edges between the current node and the target point.
        5. Returns True if the target point is successfully connected to the graph, otherwise returns False.

        @return: True if the target point is successfully connected to the graph, otherwise False.
        """
        valid = False
        self.graph.add_node(target)
        for node in self.graph.nodes():
            d = RRTSolver.dist(node, target)
            if node == target or node == self.start:
                continue
            flag=True
            for i in range(len(self.scene.robots)):
                point_node = Point_2(FT(node[2*i]), FT(node[2*i+1]))
                point_target = Point_2(FT(target[2*i]), FT(target[2*i+1]))
                edge = (Segment_2(point_node, point_target))
                for robot_test in self.scene.robots:
                    if not self.collision_detection[robot_test].is_edge_valid(edge=edge):
                        flag = False
            if not flag:
                continue
            if self.checkEdge(node,target):
                self.graph.add_edge(u_of_edge=node, v_of_edge=target, weight=d)
                self.graph.add_edge(u_of_edge=target, v_of_edge=node, weight=d)
                valid = True


        return valid

    def buildGraph(self):
        """
        The method initializes the graph, adds the start node,
        and connects additional nodes to form landmarks by calling the method `growGraph`

        The method performs the following steps:
        1. Initializes a directed graph.
        3. Grows the graph by connecting nodes to form landmarks using the `growGraph` method.
        """
        self.graph = nx.DiGraph()
        self.graph.add_node(self.start,label='start',cost=0)
        self.growGraph(count=self.numLandmarks)
        
    def growGraph(self, count: int):
        """
        Expands the graph, expansion method depends on which algorithm is running.

        @param count: An integer specifying the number of nodes to connect.

        The method performs the following steps(iteratively):
        1. Samples a valid point in the configuration space.
        2. Finds the nearest neighbor to the sampled point in the graph.
        3. Steers from the nearest neighbor towards the sampled point.
        4. If steering is successful, 
        and the edge between nearest neighbor and the new point is collision-free:
            - Adds the new point to the graph with it's cost.
            - If RRT* algorithm is used, performs an RRT* specific step.
            - Otherwise, performs RRT step - adds edges between the nearest neighbor and the new point.
        5. Iterates until the desired number of nodes are connected.
        """
        nodes_connected = 0
        while nodes_connected < count:
            sample = self.sampleValidPoint()
            nearestNeighbor, d = self.findNearestNeighbor(query=sample)
            ret = self.steer(near=nearestNeighbor, rand=sample, d=d)
            if ret is None:
                continue
            if self.checkEdge(nearestNeighbor,ret):
                self.graph.add_node(ret,cost=self.graph.nodes[nearestNeighbor]['cost'] + d)
                if self.is_star!=0:
                    self.rrtStarStep(ret,nearestNeighbor)     
                else: # RRT regular step
                    self.graph.add_edge(u_of_edge=nearestNeighbor, v_of_edge=ret, weight=RRTSolver.dist(p=nearestNeighbor, q=ret))
                    self.graph.add_edge(u_of_edge=ret, v_of_edge=nearestNeighbor, weight=RRTSolver.dist(p=nearestNeighbor, q=ret))
                nodes_connected += 1
                if nodes_connected%RRTSTAR_MODULU_FULL_ITERATION==0:
                    print(f"{nodes_connected} landmarks connected")
               
    
    def getXNear(self,query):
        """
        Generates XNEAR to a query point in the graph for the RRT* algorithm.

        @param query: A Point_d representing the query point.

        The method performs the following steps:
        1. Determines the logarithmically scaled threshold for early stopping based on the number of nodes in the graph.
        2. Shuffles the list of nodes in the graph for randomness.
        3. Determines the value of THETA based on the threshold,
           the number of landmarks, and the dimensionality of the space.
        4. Determines the value of k based on the number of nodes in the graph.
        5. Iterates through the first k nodes in the shuffled list and calculates their distance to thequery point.
        6. If the distance is less than THETA, adds the node to XNEAR.

        @return: A list of nodes represents XNEAR.
        """
        x_near = []
        n = len(self.graph.nodes())
        log_n = int(math.log(n))
        shuffled_nodes = list(self.graph.nodes())
        random.shuffle(shuffled_nodes)
        THETA = (log_n/self.numLandmarks)*n*(2**self.dim) # monotnic increasing function of n to calculate theta
        k = n if (n%RRTSTAR_MODULU_FULL_ITERATION==0 or n<=RRTSTAR_FULL_ITERATION_LIMIT) else 5*log_n
        for i in range(k): 
            d = RRTSolver.dist(shuffled_nodes[i], query)
            if THETA > d:
                x_near.append(shuffled_nodes[i])

        return x_near
        
        
    def checkEdge(self,p,q):
        """
        Checks the validity of an edge connecting two points in the configuration space

        @param p: The starting point of the edge
        @param q: The ending point of the edge

        The method performs the following steps:
        1. Converts the starting and ending points to lists of Point_2 objects.
        2. Divides the edge into multiple segments by linear interpolation.
        3. Checks each sampled point along the edge if there are robots that's collide.
        4. If any sampled point violates collision constraints, the edge is considered invalid.

        @return: True if the edge is collision-free, False otherwise
        """
        p = conversions.Point_d_to_Point_2_list(p)
        q = conversions.Point_d_to_Point_2_list(q)
        t_values = np.linspace(0, 1, 100)
        qList , pList = [] , []
        for i in range (len(p)):
            pList.extend([p[i].x().to_double(),p[i].y().to_double()])
            qList.extend([q[i].x().to_double(),q[i].y().to_double()])
        sampled_points = [(1-t)*np.array(pList) + t*np.array(qList) for t in t_values]
        is_valid = True
        for ret in sampled_points:
            for i,robot in enumerate(self.scene.robots):
                if not is_valid:
                    break
                radius = robot.radius.to_double()
                for j in range(len(self.scene.robots)):
                    if j==i:
                        continue
                    distance = math.sqrt((ret[2*j] - ret[2*i]) ** 2 + (ret[2*j+1] - ret[2*i+1]) **2 )
                    if distance<= 2*radius:
                        is_valid=False
        return is_valid
        
        

    def findNearestNeighbor(self, query: tuple):
        """
        Finds the nearest neighbor to a given query point in the graph

        @param query: A tuple representing the query point for which the nearest neighbor is to be found

        The method performs the following steps:
        1. Retrieves a list of nodes from the graph and shuffles it for randomness.
        2. Calculates the logarithmically scaled threshold for early stopping based on the number of nodes.
        3. Iterates through the shuffled list of nodes, stopping early if the number of iterations exceeds the threshold.
        4. Calculates the distance between each node and the query point and updates minimum if relevant.

        @return: A tuple containing the nearest point it's distance to the query point.
        """
        nearestPoint = None
        minDist = math.inf
        nodes = list(self.graph.nodes())
        n = len(nodes)
        log_n = 5*int(math.log(n))
        
        random.shuffle(nodes)
        for i,node in enumerate(nodes):
            if n>RRT_FULL_NEIGHBOR_SEARCH_LIMIT and i>log_n:
                break
            d = RRTSolver.dist(node, query)
            if minDist > d:
                minDist = d
                nearestPoint = node
            
        
        return nearestPoint, minDist


    def steer(self, near: tuple, rand: tuple, d: float):
        """
        Steers towards a target point from a nearby point within a specified distance

        @param near: A tuple representing the nearby point to steer from
        @param rand: A tuple representing the target point to steer towards
        @param d: The distance between near and rand

        The method performs the following steps:
        1. If the distance between the nearby and target points exceeds a threshold,
           limits the target point to be within the threshold distance.
        2. Finds the first valid point along the vector from the nearby point to the limited target point.
        3. Constructs an edge between the nearby point and the first valid point and checks for collision validity.
        4. If the edge is collision-free for all robots, returns the first valid point.
        5. Otherwise, returns None.

        @return: A valid point if steering is successful, otherwise returns None.
        @rtype: : Point_d
        """
        if d > self.eta:
            rand = self.findLimitedDistanceOnEdge(p=near, q=rand)
        new = self.findFirstValidPointOnVector(p=near, q=rand)
        edge = []
        flag = True
        for i, robot  in enumerate(self.scene.robots):
            point_i = Point_2(FT(near[2*i]), FT(near[2*i+1]))
            edge= Segment_2(point_i, new[i])
            if not self.collision_detection[robot].is_edge_valid(edge=edge):
                flag=False
                break
        if flag:
            return conversions.Point_2_list_to_Point_d(new)
        
        return None

    def findFirstValidPointOnVector(self, p: tuple, q: tuple):
        """
        Finds the first valid point along the vector from the start point to the end point

        @param p: A tuple representing the start point of the vector
        @param q: A tuple representing the end point of the vector

        The method performs the following steps:
        1. Converts the start and end points to lists of Point_2 objects.
        2. Iterates through each dimension of the points and calculates the vector between them.
        3. Determines the number of samples along the vector based on the distance and offset of the robot.
        4. Samples points along the vector and checks each sample for collision validity.
        5. Returns the last valid point encountered along the vector.

        @return: A list of Point_2 containing the first valid point encountered along the vector.
        """
        pList, qList = conversions.Point_d_to_Point_2_list(p), conversions.Point_d_to_Point_2_list(q)
        lastValidPoint = pList
        for i in range(len(pList)):
            x1 = pList[i].x().to_double()
            y1 = pList[i].y().to_double()

            x2 = qList[i].x().to_double()
            y2 = qList[i].y().to_double()


            dx = x2 - x1
            dy = y2 - y1

            robot = self.scene.robots[i]
            offset = self.collision_detection[robot].offset
            sample_count = int(
                (math.sqrt(dx ** 2 + dy ** 2) + (robot.radius.to_double() + offset.to_double())) / (
                    2 * offset.to_double()) + 1)
            
            for j in range(sample_count + 1):
                alpha = (sample_count - j) / sample_count
                beta = j / sample_count
                x = alpha * x1 + beta * x2
                y = alpha * y1 + beta * y2
                point = Point_2(FT(x), FT(y))
                if not self.collision_detection[robot].is_point_valid(point):
                    break
                else:
                    lastValidPoint[i] = point

        return lastValidPoint


    def findLimitedDistanceOnEdge(self, p: tuple, q: tuple):
        """
        Limits the distance between two points along an edge to a specified threshold

        @param p: A tuple representing the start point of the edge
        @param q: A tuple representing the end point of the edge

        The method performs the following steps:
        1. Calculates the vector representing the edge from the start to the end point.
        2. Scales the vector to limit its magnitude to the specified threshold.
        3. Computes the new end point by adding the scaled vector to the start point.
        4. Returns the new end point with limited distance along the edge.

        @return: A Point_d containing the new end point with limited distance along the edge.
        """
        new_point = []
        pList, qList = conversions.Point_d_to_Point_2_list(p), conversions.Point_d_to_Point_2_list(q)
        for i in range(len(pList)):
            pq = (qList[i].x().to_double() - pList[i].x().to_double(),
                qList[i].y().to_double() - pList[i].y().to_double())
            
            magnitude_pq = math.sqrt(pq[0]**2 + pq[1]**2)
            scaled_pq = (self.eta / magnitude_pq) * pq[0], (self.eta / magnitude_pq) * pq[1],
            new_point.append(Point_2(FT(pList[i].x().to_double() + scaled_pq[0]), FT(pList[i].y().to_double() + scaled_pq[1])))
       
        return conversions.Point_2_list_to_Point_d(new_point)

    @staticmethod
    def dist(p, q):
        """
        Calculates the Euclidean distance between two points in unknown dimensional space.

        @param p: A Point_d representing the coordinates of the first point.
        @param q: A Point_d representing the coordinates of the second point.

        The function computes the Euclidean distance between the two points by:
        1. Converting the input points to lists of Point_2 objects.
        2. Iterating through each dimension of the points and calculating the squared differences.
        3. Summing the squared differences and returning the square root of the sum.

        @return: The Euclidean distance between the two points.
        """
        pList, qList = conversions.Point_d_to_Point_2_list(p), conversions.Point_d_to_Point_2_list(q)
        d= 0
        for i in range(len(pList)):
            d+= ((pList[i].x().to_double() - qList[i].x().to_double()) ** 2 + \
                    (pList[i].y().to_double() - qList[i].y().to_double()) ** 2 ) 

        return d**0.5

    @staticmethod
    def get_arguments():
        return {'numLandmarks': ('numLandmarks', 3000, int), 'eta': ('eta', 10, float),'is_star': ('is_star', 0 , int)}

    @staticmethod
    def RRTStart(scene: Scene, numLandmarks: int, eta: float, withGui: bool,is_star=0,output="output.txt"):
        """
        Runs the RRT (Rapidly-exploring Random Tree) algorithm for path planning in a given scene

        @param scene: a Scene type object representing the environment where the RRT algorithm will be executed
        @param numLandmarks: integer value for the number of landmarks to pick
        @param eta: float value for the step size parameter used by the RRT algorithm to control the maximum distance between nodes
        @param withGui: boolean flag for displaying GUI during the execution of the algorithm
        @param is_star: optional integer flag indicating whether to use the RRT* variant of the algorithm (1 for RRT*, 0 for standard RRT). Default is 0

        The function initializes the scene, calls the solver function to run the RRT algorithm,
        displays the GUI if withGui is True, and writes the result to an output file.
        Runs the RRT algorithm for path planning in a given scene.
        """
        
        if eta == math.inf:
            for robot in scene.robots:
                if 5*robot.radius.to_double() <eta:
                    eta = 5*robot.radius.to_double()
        
        solver = RRTSolver(numLandmarks=numLandmarks, eta=eta,is_star=is_star)
        solver.load_scene(scene=scene)
        
        # solve scene
        if withGui:
            # Start solver_viewer with the scene and solver objects (the scene isn't solved yet)
            start_gui(scene, solver)
            return
        
        paths = solver.solve()
        if solver.cost == 0:
            print('No path found')
            with open(output, 'w') as f:
                f.write("No path found")
                f.close()
            return

        result, _ = verifyPaths(scene=scene, paths=paths)
        with open(output, 'w') as f:
            if not result:
                print("No path found")
                f.write("No path found")
                f.close()
            else:
                robots_points = [paths.paths[robot].points for robot in scene.robots]
                for points in zip(*robots_points):
                    f.write(' '.join('{} {}'.format(point.location[0], point.location[1]) for point in points) + '\n')
                f.write(f"\nCOST {solver.cost}")
                f.close()