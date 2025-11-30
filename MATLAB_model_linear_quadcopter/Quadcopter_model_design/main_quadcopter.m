clear all
close all
clc

%this is the file for the design of the Quadcopter with controller

% initialise the parameters of the system qnd discretisation time
quadcopterParameters

% obtain linear model of the plant in continous time and discretise at Ts
plant_quadcopter

% design linear model of the controller in continous and discrete time
controllerdesign

% compute closed loop dynamics in discrete time
closed_loop

