% Plant_quadcopter

%%%%%%%%%%%%%%%%%%%%%%%%%%% AUTOMATIC CALC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %Plant transfer function
% P_tf = tf([1], [Ix,0,0]) ; %P_tf(s) = 1/(Ix * s^2) from eq. (21) in booklet TM.
% 
% figure
% rlocus(P_tf) %K>=0
% title ('rlocus tf Ix for K>=0')
% figure
% rlocus(-P_tf) %K<=0
% title ('rlocus tf Ix for K<=0')
% 
% 
% % Discretisation
% 
% P_tfd = c2d(P_tf, Ts, 'zoh');
% [num_P_tfd, den_P_tfd] = tfdata(P_tfd, 'v');
% 
% %State-Space of the plant
% p_d_ss = ss(P_tfd)
% [Adp,Bdp,Cdp,Ddp] = ssdata(p_d_ss)
% 
% % step response of discretised system without controller
% figure
% step(P_tfd)
% title("Step response of Discretised system")


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MANUAL CALC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% attitude
% state-space representation
Ap_x = [0  0 ; 1 0] ;
Bp_x = [1/Ix ; 0] ;
Cp_x = [0 1] ;
Dp_x = 0 ;

P_ss_x = ss(Ap_x, Bp_x, Cp_x, Dp_x) ;

% discretisation
P_ss_x_d = c2d(P_ss_x, Ts, 'zoh') ;

[Adp_x,Bdp_x,Cdp_x,Ddp_x] = ssdata(P_ss_x_d)


%altitude
Ap_z = Ap_x ;
Bp_z = [1/mass ; 0] ;
Cp_z = Cp_x;
Dp_z = Dp_x ;

P_ss_z = ss(Ap_z, Bp_z, Cp_z, Dp_z) ;

% discretisation
P_ss_z_d = c2d(P_ss_z, Ts, 'zoh') ;

[Adp_z,Bdp_z,Cdp_z,Ddp_z] = ssdata(P_ss_z_d)



