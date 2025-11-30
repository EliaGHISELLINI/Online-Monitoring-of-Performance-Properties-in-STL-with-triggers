function [overshoot_bound, rise_time_t, rise_time_bound, set_time_t, set_time_bound, ss_err, APE, MPE, RPE, PDE, trigger_ESA ] = input_specs();
% function to provide the specifications of the performances on the
% quadcopter

% overshoot
overshoot_bound = 0.02; % percentage

% rise time
rise_time_t = 1.0; % seconds
rise_time_bound = 0.080; % percentage

% settling time
set_time_t = 2.0; % seconds
set_time_bound = 0.025; % percentage

% steady-state error
ss_err = 0.005; % percentage

% absolute percentage error
APE = 0.025; % percentage

% maximum percentage error
MPE = 0.025; % percentage

% relative percentage error
RPE = 0.025; % percentage

% percentage deviation error
PDE = 0.025; % percentage

% trigger ESA properties
trigger_ESA = true;

end
