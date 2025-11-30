#How to use the quadcopter linear model:

The folder "Quadcopter_model_design" contains the files to set up the model, and the Simulink model (without the input parameters)
The folder "Specifications_and_inputs" contains the files to provide the specifications on performance, and generate the reference signal.


How to create the model:
  1. run the file "main_quadcopter.m".

How to generate the specifications and the inputs:
  1. run "specs_and_inputs.m" . This file generates also a .csv file that needs to be provided to the C code of the quadcopter in order to run the simulation and verification.
  
  
If you want to have your own input, then change the files "initialise_step_overshoot.m" and "input_generation.slx".
  
 
