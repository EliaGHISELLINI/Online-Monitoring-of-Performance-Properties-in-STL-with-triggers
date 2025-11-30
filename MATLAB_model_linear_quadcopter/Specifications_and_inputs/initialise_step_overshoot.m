function [step_1_size, step_1_time, step_2_size,step_2_time, step_3_size, step_3_time, step_4_size, step_4_time, gain] = initialise_step_overshoot();

step_1_size = 5;
step_1_time = 0.2;

step_2_size = -2.5;
step_2_time = 2.5;

step_3_size = 0.8;
step_3_time = 3.5;

step_4_size = -1.2;
step_4_time = 6.0;

gain = 1;

end