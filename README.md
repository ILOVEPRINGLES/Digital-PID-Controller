# Digital-PID-Controller
The objective of this project is to design an implement a digital PID controllers (basic and anti-windup) to control the position of a DC motor using concurrent programming with Pthreads.

# Part A: Basic PID controller
## Theory

![1](https://user-images.githubusercontent.com/96636782/161468853-e82279c7-9904-4810-abe9-d00adbab3e5b.png)
Proportional Controller:

![p1](https://user-images.githubusercontent.com/96636782/161469318-e3fe02f6-a799-417c-861b-e99f76241834.png)

![proportional](https://user-images.githubusercontent.com/96636782/161469223-d74f8a6c-69f6-4990-a017-55f66c5b2e9b.png)

PID Controller:
![PID](https://user-images.githubusercontent.com/96636782/161469374-df9359ec-628d-4f4c-b459-09bc4f990db9.png)
The difference equation of integral and derivative parts are obtained based on forward rectangular rule:

![inte](https://user-images.githubusercontent.com/96636782/161469547-6fac34f8-6054-45e8-a5a1-8e6209c6e112.png)

![dera](https://user-images.githubusercontent.com/96636782/161469562-d9dccad6-b5bd-40e8-ac60-acfff5daa45b.png)

# Part B: Anti-windup PID controller
An anti-windup control scheme is utilized to overcome the problem of integrator windup.
## Theory
![anti](https://user-images.githubusercontent.com/96636782/161469709-5c02b7cf-28b4-46ee-9c7d-5e99e2eb1b40.png)
![act](https://user-images.githubusercontent.com/96636782/161469791-4bb9c92e-43e2-4f47-9e0e-0a563c9e4896.png)

New intgrator equation:


![newI](https://user-images.githubusercontent.com/96636782/161469828-2bb7db00-36d7-4ee2-b936-dec9ca63ec74.png)

# Part C: further extension
In previous steps, the control algorithm is set to be running for a certain amount of period only when all the settings are static. By removing the mutex application, the algorithm can be running continuously while the settings such as Ti, Td, Kp, N, etc, can be manipulated interactively and immediately displayed on the plot.
