%specifications
t95 = 1 ; %sec - 4*tau          
D = 1/100 ; %overshoot
%dominant poles
m = -log(D)/sqrt(pi^2+log(D)^2) ; %damping ratio
w0 = 4/(m*t95) ;

dp = -m*w0 + j*w0*sqrt(1-m^2) ; %dominant pole

figure
T = tf([w0^2], conv([1,-dp],[1,-conj(dp)])) %targetted transfer function
step(T,5) %step response of T(s)
title ('target tf')


%%%%%%%%%%%%%%%%%%%%%%%%%%% PID/I-PD DESIGN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---------------------------------
% --- PID Controller design - by TM
% ---------------------------------

fp = -15 ; %fast pole - to be added to the poles of the transfer function
q = conv([1, -fp], [1, 2*m*w0, w0^2]) ; %denominator of the closed loop 

%-----------------
%-- x axis - Ix

Kix = q(4)*Ix ; %Ki = q0*Ix
Kpx = q(3)*Ix  ; %Kp = q1*Ix
Kdx = q(2)*Ix  ; %Kd = q2*Ix

%-----------------
%-- y axis - Iy

Kiy = q(4)*Iy ; 
Kpy = q(3)*Iy ; 
Kdy = q(2)*Iy ;  

%-----------------
%-- z axis - vertical

Kiz = q(4) ;  
Kpz = q(3) ;  
Kdz = q(2) ;  

% -----------------------------------
% --- PID Controller design - by Elia
% -----------------------------------
%FIXME - the procedure seems to produce results that still have

%dp
% re_dp = real(dp);
% im_dp = imag(dp);
% numerator_tf = P_tf.Numerator{1} ;
% denominator_tf = P_tf.Denominator{1} ;
% tf_in_dp = polyval(numerator_tf, dp)/ polyval(denominator_tf, dp); %systf is defined in file sys_equations.m %(1/mass) / (dp^2 + b*dp + k);
% 
% X1 = 1/(2*im_dp)*imag(-1/tf_in_dp) - 1/(2*re_dp)*real(-1/tf_in_dp);
% X2 = 1/(2*im_dp)*imag(-1/tf_in_dp) + 1/(2*re_dp)*real(-1/tf_in_dp);
% num_L = -conv(numerator_tf,[1,-2*re_dp,re_dp^2+im_dp^2]);
% den_L = conv([0, 0, 2*re_dp, 0],denominator_tf) + conv(numerator_tf,[0, 2*re_dp*X2, 0 , -2*re_dp*(re_dp^2+im_dp^2)*X1]);
% 
% L = tf(num_L, den_L) %fictional transfer function
% 
% figure
% rlocus(L)
% title('positive Kp')
% 
% figure
% rlocus(-L)
% title('negative Kp')
% 
% 
% %choose values for the PID
% Kp_E = 0.7;
% Ki_E = -(re_dp^2+im_dp^2)*(Kp_E/(2*re_dp)+X1);
% Kd_E = -Kp_E/(2*re_dp)+X2;


%%%%%%%%%%%%%%%%%%%%%%%%%%% PID Transfer function %%%%%%%%%%%%%%%%%%%%%%%%%
 
% Ki = Ki_E ;
% Kp = Kp_E ;
% Kd = Kd_E ;
% 
% % add (1+epsilon*s) below the derivative part to make it proper
e = 1e-2 ;
% C_tf = tf([Kd + Kp*e,Kp + Ki*e,Ki],[e,1,0]);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PID DISCRETISATION %%%%%%%%%%%%%%%%%%%%%%%%%

%discretisaton of the controller

% C_tfd = c2d(C_tf, Ts, 'tustin'); %better method to discretise the system
% [num_C_tfd, den_C_tfd] = tfdata(C_tfd, 'v');
% 
% %State-Space of the PID through realisation
% c_d_ss = ss(C_tfd)
% [Adc,Bdc,Cdc,Ddc] = ssdata(c_d_ss)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% I-PD DISCRETISATION %%%%%%%%%%%%%%%%%%%%%%%%

% for attitude control

%I transfer function - controller 1
C1_x_tf = tf([Kix],[1,0]);

%PD transfer function  - controller 2
C2_x_tf = tf([Kdx + e*Kpx,Kpx],[e,1]);

%discretisaton of the controller 1
C1_x_tfd = c2d(C1_x_tf, Ts, 'tustin');
[num_C1_x_tfd, den_C1_x_tfd] = tfdata(C1_x_tfd, 'v');

%discretisaton of the controller - controller 2
C2_x_tfd = c2d(C2_x_tf, Ts, 'tustin'); 
[num_C2_x_tfd, den_C2_x_tfd] = tfdata(C2_x_tfd, 'v');

%State-Space of through realisation - controller 1
c1_x_d_ss = ss(C1_x_tfd) ; 
[Adc1_x,Bdc1_x,Cdc1_x,Ddc1_x] = ssdata(c1_x_d_ss) % the matrices are 1*1

%State-Space through realisation - controller 2
c2_x_d_ss = ss(C2_x_tfd) ; 
[Adc2_x,Bdc2_x,Cdc2_x,Ddc2_x] = ssdata(c2_x_d_ss) % the matrices are 1*1


% for altitude control

%I transfer function - controller 1
C1_z_tf = tf([Kiz],[1,0]);

%PD transfer function  - controller 2
C2_z_tf = tf([Kdz + e*Kpz,Kpz],[e,1]);

%discretisaton of the controller 1
C1_z_tfd = c2d(C1_z_tf, Ts, 'tustin');
[num_C1_z_tfd, den_C1_z_tfd] = tfdata(C1_z_tfd, 'v');

%discretisaton of the controller - controller 2
C2_z_tfd = c2d(C2_z_tf, Ts, 'tustin');
[num_C2_z_tfd, den_C2_z_tfd] = tfdata(C2_z_tfd, 'v');

%State-Space of through realisation - controller 1
c1_z_d_ss = ss(C1_z_tfd) ; 
[Adc1_z,Bdc1_z,Cdc1_z,Ddc1_z] = ssdata(c1_z_d_ss) % the matrices are 1*1

%State-Space through realisation - controller 2
c2_z_d_ss = ss(C2_z_tfd) ; 
[Adc2_z,Bdc2_z,Cdc2_z,Ddc2_z] = ssdata(c2_z_d_ss) % the matrices are 1*1


%%%%%%%%%%%%%%%%%%%%%%%%%%% I-PD State-Space %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The dicrete ss representation of the I-PD is not straightforward, due to
% the different inputs to I and PD.
% The representation proposed is based on the following system:
%
%       xi(k+1) = Ai * xi(k) + Bi * err(k)
%       xpd(k+1) = Apd * xi(k) + Bpd * yp(k)
%       yc(k) = Ci * xi(k) + Di * err(k) - ( Cpd * xpd(k) + Dpd * ypd(k) )
%
% with: 
% yp = output of the plant
% err(k) = input - yp , at time k
% xi(k) = state of the I part , at time k
% xpd(k) = state of the PD part , at time k
% 
% the state is supposed to be (xi ; xpd)
% the input is supposed to be (input ; yp)


% % Resulting matrices for the controllers - also generated in the code
% input_controller.m of the verification framework
% 
% Adcx = [Adc1_x 0 ; 
%         0 Adc2_x] ;
% 
% Bdcx = [Bdc1_x 0 ; 
%         0 Bdc2_x] ;
% 
% Cdcx = [Cdc1_x -Cdc2_x] ; 
% 
% Ddcx = [Ddc1_x -Ddc2_x] ; 
% 
% Adcz = [Adc1_z 0 ; 
%         0 Adc2_z] ;
% 
% Bdcz = [Bdc1_z 0 ; 
%         0 Bdc2_z] ;
% 
% Cdcz = [Cdc1_z -Cdc2_z] ; 
% 
% Ddcz = [Ddc1_z -Ddc2_z] ; 
