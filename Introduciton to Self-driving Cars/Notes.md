# Introduction to Self-driving Cars

## Module 1: The requirement for Autonomy


- Driving task:
  - Perceiving the environment.
  - Planning how to reach from point A to B.
  - Controlling the vehicle.
  
- Operational Design Domain (ODD)

- What makes up a driving task?
  - __Lateral control__ - steering
  - __Longitudinal control__ - braking, accelerating
  - __Object and Event Detection and Response__ (OEDR): detection, reaction
  - __Planning__:
    - Long term
    - Short term
  - Miscellaneous: signaling with indicators, hand-waving, interacting with other drivers, etc.
  
- Autonomous Capabilities
  - Automated lateral control?
  - Automated longitudinal control?
  - __OEDR__
    - Driver supervision
    - Automatic emergency response
  - Complete vs Restricted ODD
  
- __Level 0 - No Automation__: Regular vehicles, no automation
- __Level 1 - Driving Assistance__: The autonomous system assist the driver by performing either lateral or longitudinal control tasks (Adaptive cruise control).
- __Level 2 - Partial Driving Automation__: The system perform both the control tasks, both lateral and longitudinal control (e.g, GM Super Cruise, Nissan ProPilot Assist)
- __Level 3 - Conditional Driving Automation__: The system can perform Object and Event Detection in Response to a certain degree in addition to the control tasks. However, in the case of failure the control must be taken up by the driver. The key difference between level two and three, is that the driver does not need to pay attention in certain specific situation as the vehicle can alert the driver in time to intervene.
- __Level 4 - High Driving Automation__: Handle s emergencies autonomously, driver can entirely focus on other task. Level 4 system can handle emergencies on their own but may still ask driver to take over. With this system, the passenger can check their phone or watch a movie knowing that the system is able to handle emergencies. However, level four still permits self-diving systems with a limited ODD.
- __Level 5 - Fully Autonomous__: The system is fully autonomous and its ODD is unlimited. Meaning it can operate under any condition necessary.

- Limitation of this taxonomy: ODD and safety record are more important.

- __What is perception?__:
  - We want to make sense the environment and ourselves
  - Two things:
    - identification
    - understanding motion
  - Why?
    - to inform our driving decisions
    
- __Goals for perception__:
  - Statics objects: Road and lane markings; curbs; traffic lights; road signs; construction signs, obstructions, and more.
  - Dynamic object (on road)
    - Vehicle
      - 4 wheelers (cars, truck..)
      - 2 wheelers (motorbike, bicycles...)
    - Pedestrians
  - Ego localization
    - Position
    - Velocity, acceleration
    - Orientation, angular motion
  
- __Challenges to perception__
  - Robust detection and segmentation
  - Sensor uncertainty
  - Occlusion, reflection.
  - Illumination, lens flare
  - Weather, precipitation

- __Planning__
  - Making decisions
    - Long term
      - How to navigate from NY to LA?
    - Short term
      - Can I change my lane now?
      - Can I pass this intersection?
    - Immediate
      - Can I stay on track on this curved road?
      - Accelerate or brake, by how much?

- __Rule Based Planning__:
  - What we just mentioned was rule based planning (involved decision tree) 
  - In reactive rule based planning, we have rules that take into account the current state of ego vehicle and other objects and give decision.
  - Example:
    - If there is a pedestrian on the road, stop.
    - If speed limit changes, adjust speed to match it.
    
- __Predictive Planning__:
  - Make predictions about other vehicles and how they are moving. Then use these predictions to inform our decisions.
  - Example: 
    - That car has been stopped for the last 10 seconds. It is going to be stopped for the next few seconds. 
    - Pedestrian is jaywalking.She will enter our lane by time we reach her.
  - => Rely heavily on predicting information.

## Sensors and Computing Hardware
_ __Sensor___:
  - __Sensor__: device that measures or detects a property of the environment, or change to a property
  - __Categorization__:
    - Exteroceptive : surroundings
    - Proprioceptive: internal
  - __Sensors for perception:__
    - __Camera__:
      - Essential for correctly perceiving environment
      - Comparison metrics:
        - Resolution
        - Field of view
        - Dynamic range
      - Trade-off between resolution and Field of view (FOV)?
    - __Stereo camera__:
      - Enables depth estimation from image data
    - __LIDAR__:
      - Detailed 3D scene geometry from LIDAR point cloud
      - Because it is an active sensor with it's own light sources, LIDAR are not affected by the environment lightning.
      - Comparison metrics:
        - Number of beams
        - Points per second
        - Rotation rate
        - Fiew of view
      - Upcoming: Solid state LIDAR => future of affordabe autonomous driving
    - __RADAR__:
      - Robust object detection and Relative speed estimation
      - They are particularly useful in adverse weather as they are mostly unaffected by precipitation
      - Comparison metrics:
        - Range
        - FOV
        - Position and speed accuracy
      - Configurations:
        - WFOV, short range
        - NFOV, long range
    - __Ultrasonic__:
      - Sound navigation and ranging
      - Short-range all-weather distance measurement
      - Idea for low-cost parking solution
      - Unaffected by lightning, precipitation
      - Comparison metrics:
        - Range
        - FOV
        - Cost
    - __GNSS/IMU__:
      - Global Navigation Satellite System and Inertial Measurement Units
      - Direct measure of ego vehicle states
        - position, velocity (GNSS)
          - Varying accuracies: RTK, PPP, DGPS
        - angular rotation rate (IMU)
        - acceleration (IMU)
        - heading (IMU, GPS)
    - __Wheel odometry__:
      - Track wheel velocities and orientation
      - Use these to calculate overall speed and orientation of car
        - speed accuracy
        - position drift
        
    - __Highway Analysis__:
    <p align="center"><img src="./img/highway.jpg" width=640></img></p>
    
    - __Urban Analysis__:
    <p align="center"><img src="./img/urban.jpg" width=640></img></p>
    
- __Software Architecture I High-level__
<p align="center"><img src="./img/main_module.jpg" width=640></img></p>

- __Software Architecture Environment Perception__:
<p align="center"><img src="./img/environment_perception.jpg" width=640></img></p>

- __Software Architecture Environment Mapping__:
<p align="center"><img src="./img/environment_mapping.jpg" width=640></img></p>

- __Software Architecture I Motion Planning__:
<p align="center"><img src="./img/motion_planning.jpg" width=640></img></p>

- __Software Architecture I Vehicle Controller__:
<p align="center"><img src="./img/vehicle_controller.jpg" width=640></img></p>

- __Software Architecture I System Supervisor__:
<p align="center"><img src="./img/system_supervisor.jpg" width=640></img></p>

- __Environment Representation__:
  - __Environmental Map Types__:
    - Localization of vehicle in the environment
      - Localization point cloud or feature map
    - Collision avoidance with static objects
      - Occupancy grid map
    - Path planning
      - Detailed road map
  - __Point cloud or Feature Map (Localization Map)__:
    - Collects continuous sets of LIDAR point or images, which are combined to make  a point cloud representation of the environment
    - The difference between LIDAR maps is used to calculate the movement of the autonomous vehicle
  - __Occupancy Grid__:
    - Discretized fine grain grid map (can be 2D or 3D)
    - Occupancy by a static object (tree, buildings)
    - Curbs and other non drivable surfaces (dynamic objects are removed
  - __Detailed Roadmap__:
    - 3 Methods of creation:
      - Fully Online
      - Fully Offline
      - Created Ofline and Update Online

## Module 3: Safety for Self-Driving Cars

_ __Uber Crash: Multiple Things Gone Wrong__
  - No real time checks on safety drivers
  - After the woman was detected on the road (6 sec before), the system first classifed as unknown object, then misclassified as a vehicle then a bicycle. This leads to the fact that the system ignored the information since it wasn't reliable.
  - 1.3 sec before, Volvo system tried to do emergency braking maneuver, but this system was disabled in advance to avoid system conflicts.
  
- __Basic Safety Term__:
  - Harm: refer to the physical harm to a living thing
  - Risk: describe the probability that an even occurs combine with the severity that an event can cause.
  - Safety: absence of unreasonable risk of harm
  - Hazard: potential source of unreasonable risk of harm.
  
- __Major Hazard Sources__:
  - Mechanical
  - Electrical
  - Hardware
  - Software
  - Sensor
  - Behavioral
  - Fallback
  - Cyber
    
- __NHTS: Safety Framework__
<p align="center"><img src="./img/safety_framework.jpg" width=640></img></p>

- __NHTSA: Autonomy Design__
<p align="center"><img src="./img/autonomy_design.jpg" width=640></img></p>

- __NHTSA: Testing and Crash Mitigation__
<p align="center"><img src="./img/testing_crash.jpg" width=640></img></p>

- __Waymo: Safety Levels__
<p align="center"><img src="./img/waymo_safety.jpg" width=640></img></p>

- __Waymo: Safety Processes__:
  - Identify hazard scenarios & potential mitigations.
  - Use hazard assessment methods to define safety requirements:
    - Preliminary analysis
    - Fault tree
    - Design failure Modes & Effective Analyses
  - Conduct extensive testing to make sure safety requirement are met.

- __Waymo: Level of testing to ensure safety__
  - Simulation testing
    - Test rigorously with simulation, thousand of variations, fuzzing of neighboring vehicles.
    - Close-course testing
      - Follow 28 core + 19 additional scenario compentencies on private test tracks
      - Focus on four most common crashes: 
        - Rear-end, intersection, road departure, lane change
    - Real-world driving
      - Start with smaller feet, expand steadily
      - Already testing thousands of vehicles, with more on the way.

- __GM: Safety__:
  - Address all 12 elements of NHTSA Safety Framework
  - Interactive design; Analyze -> Build -> Simulate -> Drive
  - Safety through Comprehensive Risk Management and Deep Integration
    - identify and address risks, validate solutions.
    - prioritize elimination of risks, not just mitigation.
  - All hardware, software systems meet:
    - self-set standards for performance, crash protection reliability, serviceability, security and safety.
    
- __The Dilemma__:
  - Question: How many miles (years) would autonomous vehicles have to be driven to demonstrate 95% confidence their failure rate to within 20% of the true rate of 1 fatality per 140 million km?
  - Answer: ~400 years, with a fleet of 100 vehicles traveling all the time (total ~8 billion miles)
  
- __Generic Safety Frameworks__:
  - __Fault Tree Analysis__
    - Top down deductive failure analysis
    - Boolean logic
    - Assign probabilities to fault "leaves"
    - Use logics gate to construct failure tree
  
  - __Failure Model and Effects Analyses (FMEA)__
    - Bottom up process to identify all the effect of faults in a system
    - __Failure Model__: Models or ways in which component of the system may fail
    - __Effect Analysis__: Analyzing effects of the failure models on the operaton of the system.
    
- __FMEA: Steps__
<p align="center"><img src="./img/fmea_step.jpg" width=640></img></p>


## Module 4: Vehicle Dynamic Modeling

- __Kinetic Vs Dynamic Modeling__:
  - At low speed, it is often sufficient to look only at kinematic models of vehicles (e.g: two wheeled robot, Bicycle model)
  - Dynamic modeling is more involved, but captures vehicle behavior more precisely over a wide operating range. 

- __Vectors__: Vectors are variables with both magnitude and direction 

- __Coordinate Transformation__: Conversion between Inertial frame and Body coordinates is done with translation vector and a rotation matrix

- __2D Kinematic Modeling___
  - The kinematic constraint is nonholonomic
  <p align="center"><img src="./img/2dkinematic_modeling.jpg" width=640></img></p>
  

- __Simple Robot Motion Kinematics__:
<p align="center"><img src="./img/simple_motion_kinematics.jpg" width=640></img></p>

- __Two-wheel robot kinematic model__
<p align="center"><img src="./img/two_wheeled_kinematic.jpg" width=640></img></p>

- __Kinematic Model of a Simple 2D Robot__
<p align="center"><img src="./img/kinematic_simple_2D.jpg" width=640></img></p>
  
- __Dynamic Modeling:__
  - At higher speed and slippery roads, vehicles do not satisfy no slip condition.
  - Forces such as drag, road friction govern required throttle inputs.
  - Step to build a typical dynamic model: Coordinate frames -> Lumped dynamic elements -> Free body diagram -> Dynamic equation
  
- __Full Vehicle Modeling:__
  - All components, forces and moment in 3D
    - Pitch, roll, normal forces
    - Suspension, drivetrain, component model, 
    
- __2D Dynamic - Vehicle Longitudinal Motion__
<p align="center"><img src="./img/dynamic_longitudinal_motion.jpg" width=640></img></p>


- __2D Dynamic - Vehicle Lateral Motion__
<p align="center"><img src="./img/dynamic_lateral_motion.jpg" width=640></img></p>

- __Lateral Dynamics of Bicycle Model__:
<p align="center"><img src="./img/bicycle_lateral_dynamic.jpg" width=640></img></p>

- __Tire Slip Angles__:
  - Many different tire slip models
  - For small tire slip angles, the lateral tire forces are approximated as a linear function of tire slip angle


    
