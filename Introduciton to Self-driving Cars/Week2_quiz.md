## Module 2: Graded Quiz
1. What are the differences between exteroceptive sensors and proprioceptive sensors? (Select all that apply)

- [ ] Exteroceptive sensors can determine distance traveled by the vehicle, whereas proprioceptive sensors cannot.
- [x] Proprioceptive sensors do not interact with the environment, whereas exteroceptive sensors do.
- [ ] Proprioceptive sensors are used to determine vehicle position, whereas exteroceptive sensors are used for sensing the environment.
- [ ] Proprioceptive sensors can determine distance traveled by the vehicle, whereas exteroceptive sensors cannot.
- [x] Exteroceptive sensors can determine obstacle size and distance away, whereas proprioceptive sensors cannot.

2. Which of the following exteroceptive sensors would you use in harsh sunlight?
- [x] Sonar
- [ ] Lidar
- [ ] Cameras
- [x] Radar

3. Why is synchronization and timing accuracy important in the self driving system? Choose the primary reason.

- [ ] Synchronization is important to ensure that sensors measure the environment at the same time.
- [ ] Synchronization is important to check sensor failure.
- [ ] Synchronization is important to ensure organized computation.
- [x] Synchronization is important to ensure correct sensor fusion.


4. Your autonomous vehicle is driving on the German autobahn at 150 km/h and you wish to maintain safe following distances with other vehicles. Assuming a safe following distance of 2s, what is the distance (in m) required between vehicles? Round your answer to 2 decimal places.

- __83.33__

5. Using the same speed of 150 km/h, what is the braking distance (in m) required for emergency stops? Assume an aggressive deceleration of 5 m/s^2. Round your answer to 2 decimal places.
- __173.61__

6. Suppose your vehicle was using long range cameras for sensing forward distance, but it is now nighttime and the images captured are too dark. Which of the following sensors can be used to compensate?

- [ ] IMU
- [x] Radar
- [x] Lidar
- [ ] Sonar


7. What are the differences between an occupancy grid and a localization map? (Select all that apply)

- [x] An occupancy grid uses a dense representation of the environment, whereas a localization map does not need to be dense.
- [ ] The occupancy grid only contains static objects, while the localization map contains only dynamic objects.
- [ ] The localization map uses only lidar data, whereas the occupancy grid can use both lidar and camera data.
- [x] The localization map is primarily used to estimate the vehicle position, whereas the occupancy grid is primarily used to plan collision free paths.


8. The vehicle steps through the software architecture and arrives at the controller stage. What information is required for the controller to output its commands to the vehicle?

- [x] Planned paths
- [x] Vehicle state
- [ ] Environment maps
- [ ] Locations of obstacles and other vehicles


9. What is (are) the role(s) of the system supervisor? (Select all that apply)

- [x] To ensure that the maps update at the correct frequencies
- [ ] To ensure that the controller outputs are within operating range
- [x] To ensure that the sensors are working correctly
- [ ] To ensure that the planned paths are collision free


10. Which of the following tasks should be assigned to the local planner?

- [x] Planning to avoid a parked car in the ego vehicle's lane
- [ ] Planning a lane change to turn left
- [ ] Planning a route to a destination
- [ ] Planning a merge onto the highway

11. What common objects in the environment appear in the occupancy grid?

- [ ] Traffic lights
- [ ] Other moving vehicles
- [x] Parked vehicles
- [ ] Lane boundaries

12. Which of the following maps contain roadway speed limits?

- [ ] Occupancy grid
- [ ] Localization map
- [x] Detailed roadmap
