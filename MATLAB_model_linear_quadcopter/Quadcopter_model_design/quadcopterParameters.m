%run("QuadcopterParameters.m")   %TM

%-------------------
% Ts = 1/4e3; %sampling time, sec
Ts = 0.01 ; % [s], sampling time - reduced to 100Hz due to compliance to SoA sensors
sizeSv = 12 %size of parameters
%-------------------
mass = 1 %mass, kg
Ix = 4.85e-3 %inertia on x axis, kg m^2
Iy = Ix %inertia on y axis, kg m^2
Iz = 8.81e-3 %inertia on z axis, kg m^2
Inertia = diag([Ix,Iy,Iz]); %inertia matrix
Cl = 2.92e-6 %lift coefficient, N s^2 / rad^2
Cd = 1.12e-7 %drag coefficient, N m s^2 / rad^2
d = 0.2 %arm length, m
g = 9.81 %m/s^2 
sizeSv = 12
kESC = 0.01