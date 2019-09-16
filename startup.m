% PEY Final Project Simulation
%
% Jason Wang, Arnav Goel, Miguel Teran Benalcazar, Beston Leung

%% Clear everything
clc
clear
close all

%% Add folders to the path
addpath(genpath('libraries'));                  % Simulation Libraries
addpath(genpath('scripts'));                  % Simulation Libraries

%% Load basic robot parameters from modeling and simulation example
robotParameters

%% Open the README file
edit README.md