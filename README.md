# Algorithmic Robotics and Motion Planning – Final Project - Dor Falah

This project compares three robot motion planning algorithms— PRM, RRT, and RRT*.
The project includes implementations of each algorithm, performance evaluations, and visualization of the results.

## Python executers

The following Python files are included in this project:

- `main.py`: The entry point of the project. It initializes the environment and runs the specified motion planning algorithm.

- `experiment.py`: Performs the experiments across different scenes and parameters, running each algorithm multiple times and collecting the results.
The experiment runs *with threads* , the threads amount depends on how many scenes and landmarks you wish to test - 
number of threads = num_of_scenes*len(landmarks)
The list "landmarks" is defined in the code with default value of [2000, 3500, 5000].(For the scene "3 Robots Hard Maze" I used [12500, 15000, 17500] and for "5 Robots Hard" I used [15000, 17500, 20000])

- `visualize_results.py`: Generates graphs to visualize the results of the experiments, making it easier to compare the performance of the algorithms.I've attached my full experiment graph results to my project report for your imperssion.


## Folders 

### 1. inputs
This folder includes custom scenes for running the algorithms. 
The scenes contains up to 5 robots.

Inside the folder there's another folder with all scenes images.

### 2. modules
This folder includes the next python files:
- `prm_solver.py`: Contains the implementation of the PRM algorithm which triggerd with the function PRMStart that callable from main.py.
- `rrt_solver.py`: Contains the full implementation of the RRT and RRT* algorithms which triggerd with the function RRTStart that callable from main.py

### 3. experiments
This folder includes the output json files that generated from experiment.py.
Those file are used as input to visualize_results.py in order to visualize the given experiment result.

## How to run?

### 1. main.py
    To run main.py use the following command:

    python3 main.py -scene <path_to_scene> [--with-gui 0 or 1] [--num-landmarks <number>] 
                [--k <number>] [--solver <prm|rrt>] [--star 0 or 1] [--eta <value>] 

    -scene: Path to the scene to run (required).
    --with-gui: Boolean for GUI option (0 or 1, default is 0 which means without GUI).
    --num-landmarks: Number of landmarks to connect (default is 1000).
    --k: Number of neighbors to connect with in knn to use with PRM solver (default is 15).
    --solver: Choose which solver to use (prm, rrt, default is prm).
    --eta: Eta constant value to use with RRT solver (default is infinity, in that case, eta will be calculated according to the robots properties).
    --star: Choose if you want to run the RRT* algorithm (0 or 1, default is 0 which means RRT).
    --output: Path of the output file(default is "output.txt")

    ### Examples:
    a. To run the main.py with the scene file 3_discs_custom.json inside the folder inputs, using RRT* algorithm with a GUI, and 2000 landmarks:

    python3 main.py -scene inputs/3_discs_custom.json --with-gui 1 --num-landmarks 2000 --solver rrt --star 1

    b. To run the same scene with PRM algorithm, without GUI, 2000 landmarks and k=20:

    python3 main.py -scene inputs/3_discs_custom.json --num-landmarks 2000 --solver prm --k 20

### 2. experiment.py
    To run experiment.py use the following command:

    python3 experiment.py [-output <outputfile_name>] [--laps <number_of_laps>]

    -output: Name of the output file (default is "report", .json isn't needed).
    --laps: Number of laps per scene & landmarks to test (default is 5).

    ### Example:
    To run experiment.py with 10 laps per scene & landmarks and save output to a file named experiment_results:

    python3 experiment.py -output experiment_results --laps 10

### 3. visualize_results.py
    To run visualize_results.py use the following command:

    pytho3 visualize_results.py -report <path_to_report>

    -report: Path to the report file which generated from experiment.py (required).
