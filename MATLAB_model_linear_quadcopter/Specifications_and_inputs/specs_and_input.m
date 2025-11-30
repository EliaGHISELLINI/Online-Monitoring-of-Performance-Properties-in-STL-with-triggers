%{ 
This is the file used to create the .cvs file for the framework.  
Notes on its usage:
- The specifications are hard coded ;
- the inputs are provided to a Simulink model that runs a simulation
and returns the value of the reference instant by instant;
- the value Ts needs to be obtain from the main.m file of the quadcopter
model;
- the .cvs file is produced in a way to match the inputs of the function
compiled by CoCoSim that runs the model of the quadcopter and its 
observers;
%}

addpath ('/home/elia/these_elia_ghisellini/MATLAB/quadcopter_STL_verif/input_generation');
%%%%%%%%%%%%%%%%%%%%%%% Performance Requirements %%%%%%%%%%%%%%%%%%%%%%%%%%
s_specs = struct;

[s_specs.overshoot_bound, s_specs.rise_time_t, s_specs.rise_time_bound, s_specs.set_time_t, s_specs.set_time_bound, s_specs.ss_err, s_specs.APE, s_specs.MPE, s_specs.RPE, s_specs.PDE, s_specs.trigger_ESA ] = input_specs();
%% %%%%%%%%%%%%%%%%%%%% Performance Requirements %%%%%%%%%%%%%%%%%%%%%%%%%%
% inputs used for the evaluation

s_input = struct;

[s_input.ov.step_1.size, s_input.ov.step_1.time, s_input.ov.step_2.size, s_input.ov.step_2.time, s_input.ov.step_3.size, s_input.ov.step_3.time, s_input.ov.step_4.size, s_input.ov.step_4.time, s_input.ov.gain] = initialise_step_overshoot();

%% %%%%%%%%%%%%%%%%%%%%% Obtain reference from SL %%%%%%%%%%%%%%%%%%%%%%%%%%

paramstruct = struct; paramstruct.StopTime = "10.0"; 
out = sim("input_generation.slx", paramstruct);


%% Create the .csv file to be provided to the C code


input_array = zeros((size(fieldnames(s_specs), 1) + 4) * size(out.simout.Data,1), 1);

j = 0;
for i = 1 : 15 : size(input_array,1)
    j = j + 1;
    input_array(i) = out.simout.Data(j,1) ;
    input_array(i + 1) = 0.0;
    input_array(i + 2) = s_specs.overshoot_bound;
    input_array(i + 3) = s_specs.rise_time_bound;
    input_array(i + 4) = s_specs.set_time_bound;
    input_array(i + 5) = s_specs.ss_err;
    input_array(i + 6) = s_specs.APE;
    input_array(i + 7) = s_specs.MPE;
    input_array(i + 8) = s_specs.RPE;
    input_array(i + 9) = s_specs.PDE;
    input_array(i + 10) = s_specs.trigger_ESA;
    input_array(i + 11) = int32(s_specs.rise_time_t / Ts);
    input_array(i + 12) = int32(s_specs.set_time_t / Ts);
    input_array(i + 13) = int32(1/Ts);
    input_array(i + 14) = int32(str2double(paramstruct.StopTime) / Ts);
end

writematrix(input_array(:,1), 'input_quadcopter.csv');

