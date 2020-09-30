## Module 1: Basics of 3D Computer Vision
This module introduced the richness and challenges of the self-driving motion planning problem, demonstrating a working example that will be built toward throughout this course. The focus will on defining the primary scenarios encountered in driving, types of loss functions and constraints that affect planning, as well as common decomposition of the planning problem into behavior and trajectory planing subproblems. This module introduces a generic, hierarchical motion planning optimization formulation that is further expanded and implemented throughout the subsequent modules. 

- **Behaviours**
  - Speed tracking 
  - Decelerate to Stop
  - Stay stopped
  - Yield
  - Emergency stop

- **Challenges**
  - Only covered a small subset of scenarios.
    - Focused on common cases that follow the rules of the road.
  - Edge cases make the driving task complex (e.g. lane splitting, jay walking)

- **Hierarchical Planning Introduction**
    - Driving mission and scenarios are complex problems.
    - Break them into a hierarchy of optimization problems. 
    - Each optimization problem tailored to the correct scope and level of abstraction.
    - Higher in the hierarchy means more abstraction.
    - Each optimization problem will have constraints and objective functions.
    <p align="center"><img src="./img/hierarchical_planning.jpg"></img></p><br>
    
- **Static Obstacles**   
  - Static obstacles block portions of workspace
    - Occupancy grid encoding stores obstacle locations.
  
  - Static obstacle constrains satisfied by performing collision checking
    - Can check for collisions using th swath of the vehicle's path.
    - Can also check for closet obstacle along ego vehicle' path. 
    
- **Efficiency**    
  - Path length: Minimize the arc length of a path to generate the shortest path to the goal.
  - Travel time: Minimize the time to destination while following the planned path. 

- **Reference Tracking**

- **Smoothness**:
<p align="center"><img src="./img/jerk.jpg"></img></p><br>

- **Curvature**
<p align="center"><img src="./img/culvature.jpg"></img></p><br>

- **Hierarchical Planner**
  - **Mission planner**
    - Highest level planner, focuses on map-level navigation.
    - Abstract away lower level details.
    - Can be solved with graph-based methods (Dijkstra's, A*)
    
  - **Behavior planner** 
    - Focuses on other agents, rules of the road, driving behaviors.
    - Decide when it is safe to proceed.
    - Take pedestrians, vehicles, cyclists into consideration.
    - Also looks at regulatory elements, such as traffic lights and stop signs. 
    
    - **Finite State Machines**
      - Composed of state and transitions: _states_ are based on perception of surroundings and _transitions_ are based on inputs to the driving scenarios (e.g. traffic light changing color). 
      - FSM is memoryless: transition only depend on input and current state, and not on past state sequence. 
    <p align="center"><img src="./img/finite_state_machine.jpg"></img></p><br>
    
    - **Rule-based system**
      - Rule-bases systems use a hierarchy of rules to determine output behavior.
      - Rules are evaluated based on logical predicates.
        - Higher priority rules have precedence. 
      - Example scenario with two rules:
        - Green light + intersection -> drive straight.
        - pedestrian + driving straight -> emergency stop.

    - **Reinforcement Learning**
    <p align="center"><img src="./img/reinforcement_learning.jpg"></img></p><br>
    
    
  - **Local planner**
    - Local planning generates feasible, collision-free paths and comfortable velocity profiles. 
    - Decomposed into path planning and velocity profile generation.
    
    - **Sampling-based planner**
      - Randomly sample the control inputs to quickly explore the workspace.
      - Collision checking is performed as new points are added to the explored space. 
      - Often very fast, but can generate poor-quality paths. 
      - Rapidly Exploring Random Tree (RRT)
      
    - **Variational Planner**  
      - Optimize trajectory according to cost functional
        - Contains penalties for collision avoidance and robot dynamics
      - Variational planners are usually trajectory planners, which means they combine both path planning, and velocity planning into a single step. 
      - Can be slower, and less likely to converge to a feasible solution.
    
    - **Lattice Planners**
      - Constrain the search space by limiting actions available to the robot (set of actions known as control set).
      - Layers of control actions form a graph, which can be searched using Dijkstra's.
      - Conformal lattice planner fits the control actions to the road structure.
      
# Module 2: Mapping for Planning
The occupancy grid is a discretization of space into fixed-sized cells, each of which contains a probability that it is occupied. It is a basic data structure used throughout robotics and alternative to storing full point clouds. This module introduces the occupancy grid and reviews the space and computation requirements of the data structure. In many cases, a 2D occupancy grid is sufficient; learners will examine ways to efficiently compress and filter 3D LIDAR scans to form 2D maps.

- **Ocupancy Grid**
  In the map below, we can see that the squares with trees and grass cover are labeled as one, whereas the road is labeled as zero.
  <p align="center"><img src="./img/occupancy_map.jpg"></img></p><br>

- **Range sensor**
  - 2D range sensor measuring distance to static objects.

- **Probabilistic Occupancy Grid**
  - Use to handle sensor noise, environmental noise and map uncertainties.
  - Each cell from Occupancy grid would have a specific probability, and a threshold of certainty will be used to establish occupancy.  
  - To improve robustness multiple timesteps are used to produce the current map.
  - Bayes's theorem is applied for at each update step for each cell.
  - Issue:  
    - Multiplication of numbers close to zero is hard for computers.
    - Store the log odds ratio rather than probability.
  
- **Bayesian Log Odds Single Cell Update Derivation**
  - Numerically stable.
  - Computationally efficient. 
  <p align="center"><img src="./img/bayesian_log_odds.jpg"></img></p><br>
  
- **Inverse Measurement Module**
  - The measurement model in the mapping case represents the probability of getting a certain lidar measurement, given a cell in the occupancy grid is occupied.
  - For occupancy grid updates, we need to flip this measurement model around &rightarrow; an inverse measurement model.  

- **Inverse Measurement Module with Ray Tracing**
  - Ray tracing algorithm using Bresenham's line algorithm (fast calculation)
  - Perform update on each beam from the LiDar rather then each cell on the grid:
    - Perform far fewer updates (ignores no information zone).
    - Much cheaper per operation.

- **Filtering of 3D LIDAR**
  - Downsample the number of points of a LiDAR scan to a smaller amount to make update operation run in real-time.
  - Remove objects that don't affect driving (e.g: objects above car height)
  - Remove LiDAR points that hit the ground plane (drivable surface). We can utilize segmentation to remove points of road elements. 
  - Remove Dynamic objects.
  
- **Projection of LIDAR to 2D Plane**
    - **Simple solution**:
      - Collapse all points by zeroing the Z coordinate.
      - Sum up the number of LIDAR points in each grid location:
        - More points indicated greater change of occupation of that grid cell. 

- **High Detailed Road Map**: stores all of the locations of road signs and signals which might effect the autonomous vehicle. Due to the detailed and interconnected nature of the data, an effective method is required to store all information contained within the map. 

- **Lanelet Map**: 
  <p align="center"><img src="./img/lanelet_map.jpg"></img></p><br>

  -  **Lanelet Element**: store all information connected to a small longitudinal segment of a lane on a road which it represents. 
    - Defines the following:
      - Left and right boundaries: Define the edges of a driving lane. Different operation can be performed on boundaries such as heading, curvature and center line.
      - Regulation: Elements (e.g. stop sign) and Attributes (e.g. speed limit.)
      - Connectivity to other lanelets.
    - A new lanelet is created when a new regulatory element is encountered or ends. 
    
  - **Intersection Element**: store all lanelet elements which are parts of a single intersection for simple retrieval during motion planning tasks.  This will be followed by an explanation of connectivity all of the different lanelet element.
  
- **Operations Done on Lanelets**
  - Make motion planning process simpler and more computationally efficient. 
  - Path planning through complex road networks.
  - Localize Dynamic Objects.
  - Interaction with other Dynamic Objects.
  
- **Creations of Lanelets**
  - Offline creation.
  - Online creation.
  - Offline creation with online updating. 

## Module 3: Mission Planning in Driving Environments
This module develops the concepts of shortest path search on graphs in order to find a sequence of road segments in a driving map that will navigate a vehicle from a current location to a destination. The module covers the definition of a roadmap graph with road segments, intersections and travel times, and presents Dijkstra's and A* search for identification of the shortest path across the road network.

- **Mission Planning**: The objective of autonomous driving mission is ton find the optimal path for the eagle vehicle from its current position to given destination by navigating the road network while abstracting away the lower-level details (e.g, the rules of the road, other agents in driving scenarios)

- **Graphs**: A discrete structure composed of a set of vertices denoted as V and a set of edges denoted as E. For the mission planner, each vertex in V will correspond to a given point on the road network, and each edge E will correspond to the road segment that connects any two points in the road network.

<p align="center"><img src="./img/graphs.jpg" width = 640></img></p><br>

- **Breadth First Search (BFS)**: work only with _unweighted graph_
<p align="center"><img src="./img/BFS.jpg"></img></p><br>

- **Unweighted Graph**: We assume that all road segments have equal length and ignore any factor of speed limit, traffic light, etc. 

- **Weighted Graph**: We can add edge weights to each edge in the graph, that correspond to the length of the corresponding road segment.
  <p align="center"><img src="./img/weighted_graph.jpg"></img></p><br>
  
- **Dijksra's Algorithm**
  <p align="center"><img src="./img/dijkstra_algo.jpg"></img></p><br>
  
  
  
- **Problem of Dijksras**: __Dijksra's__ algorithm required us to search almost all of the edges present in the graph, even though only a few of them were actually useful for constructing the optimal path &rightarrow; issue whe nwe scale our problem to full road network for a city.

- **Eclidean Heuristic**
  - Exploits structure of the problem.
  - Fast to calculate.
  - Straight-line distance between two vertices is a useful estimate of true distance along the graph. **h(v) = ||t-v||*

- **A* Algo**:
  <p align="center"><img src="./img/A_start_algo.jpg"></img></p><br>
  
- **Extensions to other factors**:
    - Traffic, speed limits, and weather affect mission planning.
    - Time rather than distance is better at capturing these factors.
    - Replace distance edge weights with time estimate. 
