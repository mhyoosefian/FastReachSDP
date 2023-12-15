# FastReachSDP
This repository contains the code for fast reach-SDP. 

# Reach-SDP
Reach-SDP was developed in [1] to compute a tight reachable set for dynamical systems in feedback with neural network controllers. This repository contains a code for it. Unlike the original code provided the authors of the original paper, this code is not just developed for the double integrator or the quadrotor. It works for any dynamical system in feedback with a ReLU neural network with any number and size of hidden layers. 

Moreover, this code should be faster than the original code provided in [1], as the matrices that appear in the LMI are smaller. In this code, the approach developed in [2] is used. To be more specific, instead of a single large matrix for the neural network in the LMI, we have one smaller matrix for each layer. 

# How to use the code?
The code is abundant with comments. You can choose whether to run it for the quadrotor or the double integrator. Another option is to define your system (both the dynamical system and the neural network and use the code for that (line 47). The output of the code is the variable 'eps' which includes the size of each facet of the reachable set at each time step. 

# Prerequisites
The code uses Yalmip as the modelling language. You can choose the solver by changing the variable 'solver' in the code. By default, it uses MOSEK. It is highly recommended to use MOSEK!

# To do
Currently, the code approximates the reachable set with hyper-cubes. Other polytopes can be used to have a tighter over-approximation [3]. 

# References
1- Hu, H., Fazlyab, M., Morari, M. and Pappas, G.J., 2020, December. Reach-sdp: Reachability analysis of closed-loop systems with neural network controllers via semidefinite programming. In 2020 59th IEEE Conference on Decision and Control (CDC) (pp. 5929-5934). IEEE.

2- Fazlyab, M., Morari, M. and Pappas, G.J., 2021, December. An introduction to neural network analysis via semidefinite programming. In 2021 60th IEEE Conference on Decision and Control (CDC) (pp. 6341-6350). IEEE.

3- Entesari, T. and Fazlyab, M., 2023, June. Automated reachability analysis of neural network-controlled systems via adaptive polytopes. In Learning for Dynamics and Control Conference (pp. 407-419). PMLR.
