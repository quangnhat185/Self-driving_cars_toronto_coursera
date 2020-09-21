## Module 3: Graded Quiz

1. Which from the below options is the most ACCURATE and COMPLETE definition of risk in terms of self-driving vehicles?

- [ ] Risk is any exposure to possible loss or injury
- [ ] Risk is a probability or threat of damage, injury, liability, loss, or any other negative occurrence that is caused by external or internal factors
- [x] Risk is a probability that an event occurs combined with the severity of the harm that the event can cause
- [ ] Risk is a condition in which there is a possibility of an adverse deviation from the desired or expected outcome
- [ ] None of the above

2. Which of the following are major components of an autonomous driving system? (Select all that apply)

- [x] Perception
- [x] Planning
- [ ] Adaptation
- [x] Control
- [ ] Configuration

3. What are the most common categories of autonomous vehicle hazard sources? (Select all that apply)

- [x] Electrical and mechanical
- [x] Driver inattention
- [x] Perception and planning
- [x] Malicious software
- [x] Hardware and software

4. Is the following statement TRUE or FALSE?
The safety framework to structure safety assessment for autonomous driving defined by NHTSA is MANDATORY to follow.
- [x] FALSE
- [ ] TRUE

5. Which categories are included in the safety framework to structure safety assessment for autonomous driving defined by NHTSA? (Select all that apply)

- [x] Autonomy design
- [x] Well-organized software development process
- [ ] Digital vehicle model design
- [x] Testing and crash mitigation

6. Which actions are needed to be performed in the event of an accident by an autonomous vehicle? (Select all that apply)

- [x] Alerting first responders
- [x] Data recording to a black box
- [x] Securing fuel pumps
- [x] Returning car to a safe state
- [ ] Locking all doors

7. What are the most common accident scenarios? (Select all that apply)
- [ ] Rollover
- [x] Road departure
- [x] Intersection
- [x] Rear-end
- [ ] Crosswalk
- [x] Lane change

8. What kind of safety system is described by the following definition? This system can be analyzed to define quantifiable safety performance based on critical assessment of various scenarios.

- [ ] Data driven safety
- [ ] Test driven safety
- [x] Analytical safety
- [ ] None of the above

9. According to the report by Rand Corporation, autonomous driving of 8.8 billion miles is required to demonstrate human-level fatality rate of an autonomous vehicle fleet using a 95% Confidence Interval. How many years is required to perform this testing with a fleet of 100 vehicles running 24 hours a day, 7 days a week at an average of 25 miles per hour? Your answer should be an integer.
- __400__

```
8,800,000,000 miles / 100 vehicles = 88,000,000 miles per vehicle
88,000,000 miles / 25 miles per hr = 3,520,000 hrs per vehicle
24 hours * 365 days = 8,760 hrs in a year
3,520,000 hrs / 8,760 hrs in a year = 401.8 years
```

10. Given that an autonomous vehicle failure has happened and based on this tree, what is the probability that the failure happened because of Vehicle Control Algorithm Failure OR Inadequate Car Drivers? Please give your answer with the precision of 3 decimal places.
Please use this probabilistic fault tree for your computation:

- __0.382__

11. Given that the autonomous vehicle failure has happened, and based on this tree, what is the probability that the failure happened because of Software Failure AND Extreme Weather Conditions at the same time? Please give your answer with the precision of 3 decimal places.
Please use the probabilistic fault tree from the previous question for your computation:

- __0.001__

12. A computer vision algorithm is responsible for extracting meaningful data from the onboard camera. A computer vision failure restricts the vehicle's ability to navigate the environment around it, hence a problem with this system is a serious failure. However, LiDAR and radar sense similar environment data, so a computer vision failure does not leave the vehicle completely blind. A Computer vision algorithm failure can be considered a somewhat severe failure as it decreases vehicle sensing ability and it gets a severity score of 5. This could happen regularly in low light situations, hence the occurrence number is assigned 4. Computer vision algorithm failure is fairly detectable in majority of the situations, so the detectability score is 3. What is the risk priority number for a Computer vision algorithm failure according to FMEA and based on the description above? Your answer should be an integer.

- __60__

13. There are failures listed below. Which failures should we focus on solving first according to FMEA? 

- [x] Vehicle driving onto a gravel road (risk priority score of 400)
- [ ] GPS synchronization failure (risk priority score of 300)
- [ ] Computer vision algorithm failure (risk priority score of 60)
- [ ] Vehicle motion prediction failure (risk priority score of 150)

14. Which of the following options is the most ACCURATE and COMPLETE definition of functional safety in terms of self-driving vehicles?

- [ ] Functional safety is the process of avoiding unreasonable risk of harm to a living thing.
- [x] Functional safety is the detection of a potentially dangerous condition resulting in the activation of a protective or corrective device or mechanism to prevent hazardous events arising or providing mitigation to reduce the consequence of the hazardous event
- [ ] Functional safety is a deterministic algorithm outlining the procedures that are carried out to prevent hazardous events from happening or minimizing the harm caused by hazardous events to the vehicle passengers and third parties involved in the situation
- [ ] Functional safety is a part of the vehicle operation management aimed to minimizing hazards, risks, accidents and near misses
- [ ] None of the above

15. Which of the following standards defines functional safety terms and activities for electrical and electronic systems within motor vehicles?

- [ ] ISO/TC 204
- [ ] ISO 39001
- [ ] ISO/PAS 21448
- [x] ISO 26262
- [ ] None of the above
