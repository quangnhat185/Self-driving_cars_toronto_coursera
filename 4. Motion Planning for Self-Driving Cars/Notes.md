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
      
