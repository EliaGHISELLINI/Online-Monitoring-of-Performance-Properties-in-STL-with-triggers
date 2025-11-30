
% Closed loop with cpntroller and linear model drone

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%CLOSED LOOP STATE-SPACE

% for PID
% Adcl= [Adc, -Bdc.*Cdp; Bdp.*Cdc, Adp-Bdp.*Ddc.*Cdp]
% 
% disp('eigenvalues of Adcl')
% disp(eig(Adcl))
% disp('absolute value of eigenvalues of Adcl')
% disp(abs(eig(Adcl)))

% for I-PD
Adcl_x = [Adc1_x    0   -Bdc1_x*Cdp_x ;
        0   Adc2_x    Bdc2_x*Cdp_x ; 
        Bdp_x*Cdc1_x    -Bdp_x*Cdc2_x   Adp_x-Bdp_x*Ddc1_x*Cdp_x-Bdp_x*Ddc2_x*Cdp_x ]

eig(Adcl_x) 
abs(eig(Adcl_x))

% for I-PD
Adcl_z = [Adc1_z    0   -Bdc1_z*Cdp_z ;
        0   Adc2_z    Bdc2_z*Cdp_z ; 
        Bdp_z*Cdc1_z    -Bdp_z*Cdc2_z   Adp_z-Bdp_z*Ddc1_z*Cdp_z-Bdp_z*Ddc2_z*Cdp_z ]

eig(Adcl_z) 
abs(eig(Adcl_z))
