# DPKF
This repository contains the code to regenerate figures and results from the following paper:
>M.H. Yoosefian Nooshabadi and L. Lessard.
"State Estimation for Linear Systems with Non-Gaussian Measurement Noise via Dynamic Programming", IEEE Conference on Decision and Control, 2025 ([arXiv](linkToArxiv)) 

"DPKF" stands for "Dynamic Programming Kalman Filter" and refers to an estimation method for linear discrete-time dynamical systems with Gaussian process noise and non-Gaussian measurement noise. The proposed method outperforms the standard Kalman filter and has comparable performance to state-of-the-art methods, while requiring significantly less computational power. It also doesn't rely on restrictive assumptions on the noise density function and only requires it to be differentiable.

## About the simulations 
In the paper, the proposed method (DPKF) has been tested on 8 different distributions, listed below:
- [Skewed normal](https://en.wikipedia.org/wiki/Skew_normal_distribution)
- [Bimodal Gaussian mixture](https://en.wikipedia.org/wiki/Mixture_distribution)
- [Gamma](https://en.wikipedia.org/wiki/Gamma_distribution)
- [Impulsive Gaussian mixture](https://en.wikipedia.org/wiki/Mixture_distribution)
- [Cauchy](https://en.wikipedia.org/wiki/Cauchy_distribution)
- [Beta prime](https://en.wikipedia.org/wiki/Beta_prime_distribution)
- [Exponential](https://en.wikipedia.org/wiki/Exponential_distribution)
- [Lévy](https://en.wikipedia.org/wiki/L%C3%A9vy_distribution)

In addition to the proposed method, four other estimators have been implemented:
- Standard Kalman filter
- Masreliez filter ([paper](https://ieeexplore.ieee.org/abstract/document/1100882))
- Maximum correntropy Kalman filter ([paper](https://www.sciencedirect.com/science/article/pii/S000510981630396X?casa_token=Y8N0SfFWiLYAAAAA:dVQfe5rrev0-yUC6_nCXLk0XEYy3VGdbefscJoVFwLyFHd8fW1UFZLHzvmK8NhqvupWWE4XMpg))
- Particle filter (MATLAB's built-in function)

## How to reproduce the results
To reproduce the results, you need to run `runMe.m`. Upon running this code, it will take the following steps
1. Fig. 1 in the paper will be generated and saved as `Fig1.pdf` in the `figs` folder.
2. A prompt will appear on the command window asking whether the simulations should be run or just the results should be generated. You have two options:
   - Enter `y`: the code will run the entire simulations. To do this, the "Optimization toolbox" needs to be installed on your machine. After      running the simulations, the code will store the data and then proceed to the next step.
   - Press any other key: the code will use the stored data and then proceed to the next step.
3. The code will print the results from Table II in the paper on the command window. Additionally, it will create a similar table to Table II and store it as `TableII.png` in the `figs` folder.
4. Finally, the code will generate Fig. 2 in the paper and save it as `Fig2.pdf` in the `figs` folder.

The code was tested on a MacBook Pro M2 machine with MATLAB R2022b. 

## Questions and Comments
For any questions or comments please feel free to contact the authors
- Mohammad Hussein Yoosefian Nooshabadi: yoosefiannooshabad.m@northeastern.edu
- Laurent Lessard: l.lessard@northeastern.edu

## Citation
To cite this paper, please use the following
```
@inproceedings{dpkf,
    title={State Estimation for Linear Systems with Non-Gaussian Measurement Noise via Dynamic Programming}, 
    author={Nooshabadi, Mohammad Hussein Yoosefian and Lessard, Laurent},
    booktitle={2025 IEEE 64th Conference on Decision and Control (CDC)},
    year={2025},
    publisher={IEEE}
}
```






  
