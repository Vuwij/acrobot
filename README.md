# PEY Group 2019093 

This repository contains source code for the walking acrobot robot

## Setup
Run `startup.m` to get the MATLAB path ready and then start the robot.slx 
simulink project and press run

Each of these folders has its own separate README with more information.

# Uploading onto the robot
Note there is a hidden hack API here (https://www.mathworks.com/matlabcentral/answers/351957-do-simulink-deploy-to-hardware-and-external-modes-work-with-arduino-uno-clones)
codertarget.arduinobase.registry.setBaudRate(gcs,230400)
