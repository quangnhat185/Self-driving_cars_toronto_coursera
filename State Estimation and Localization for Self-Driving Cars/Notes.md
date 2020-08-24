# State Estimation and Localization for Self-Driving Cars

## Module 1: Least Squares
This method of least squares, developed by Carl Friedrich gauss in 1795, is a well known technique for estimating parameter values from data. This module provides a review of least squares, for the case of unweighted observation. There is a deep connection between least squares and maximum likelihood estimator (when the observation are considered to be Gaussian random variables) and this connection is established and explained. Finally, the module develops a technique to transform the traditional "batch" least squares estimator a recursive form, suitable for online, real-time estimation application. 

### Squared Error Criterion and the Method of Least Square

_The most probable value of the unknown quantities will be that in which the sum of the square of the difference between the actually observed and the computed values multiplied by number that measure the degree of precision in a minimum.__ - Carl Friedrich Gauss

- __Assumptions__:
  - Our measurement model, y=x+v is linear
  - Measurement are equally weighted (we do not suspect that some have more noise than others)

  <p align="center"><img src="./img/least_squares.jpg" width=640></img></p>

-  __Summary__:
  - Measurement can come from sensors that have different noisy characteristics.
  - Weighted least squares let us weight each measurement according to noise variance. 


### Recursive Least Square
We've already explored the problem of computing value for some unknown but constant parameter from a set of measurement- One of our assumptions was that we had all of the data at hand. That it, we assumed that we collected a batch of measurements and we wanted to use those measurement to compute our estimated quantities of interest. 

What can we do if we have a stream of data? For example, let's say we have a multimeter that can measure resistance 10 times per second. Ideally we'd like to use as many measurements as possible to get an accurate estimate of the resistance. If we use the method of least square however, the amount of computational resources we will need to solve our normal equations will grow with the measurement vector size. 

Alternatively, we can try and use a recursive method one that keeps a running estimate of the optimal parameter for all of the measurement that we've collected up to the previous time step and then updates that estimate given the measurement at the current time step. To do this we use a recursive algorithm, incrementally updating our estimate as we go along.

- __Recursive Recursive Estimator__:
  <p align="center"><img src="./img/recursive_least_squares.jpg" width=640></img></p>
  
- __Summary__:
  - RLS produces a "running estimate" of parameters(s) for a stream of measurement.
  - RLS is a linear recursive estimator that minimizes the covariance of the parameter(s) at the current time.
  
### Least Squares and Maximum Likelihood
<p align="center"><img src="./img/maximum_likelihood.jpg" width=640></img></p>

- __The Central Limit Theorem__
  - In realistic systems like self-driving cars, there are many resources of "noise"
  - _Central Limit Theorem: When independent random variables are added, their normalized sum tends towards a normal distribution_
  - Why use method of least squares?
    1. Central Limit Theorem: sum of different errors will tend be "Gaussian"
    2. Least squares is equivalent to maximum likelihood under Gaussian noise

- __Least Squares | Some Caveats__
  - Poor measurement (e.g. outliers) have significant effect on the method of least square
  - It's important to check that the measurement roughly follow a Gaussian distribution
  - Outliers might result from people walking in the middle of a Lidar scan, or from a bad GPS signal. 

- __Summary__:
  - LS as WLS produce the same estimate as maximum likelihood assuming Gaussian noise
  - Central Limit Theorem states that complex error will tend towards a Gaussian distribution.
  - Least squares estimates are significantly affected by outliers.
