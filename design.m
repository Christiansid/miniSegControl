clear all
Ts = 0.025; 

% Simple Kalman filter for alphadot and alpha

Aa = [0 0; 1 0];
Ca = [1 0; 0 1];
R1a = [1 0; 0 0];
R2a = [8e-6 0; 0 3e-5];  % Gives about w_b = 1 rad/s for y_2 => xhat_2
                            % and w_b = 100 rad/s for y_1 => xhat_1
                            
Ka = lqe(Aa,eye(2),Ca,R1a,R2a) % Kalman gain
kalman1 = ss(Aa-Ka*Ca,Ka,eye(2),0);
IMU_Kalman_D = c2d(kalman1,Ts,'foh')

% Simple Kalman filter for thetadot and theta

At = [0 0; 1 0];
Ct = [0 1];

R1t = [50^4 0; 0 0];
R2t = [1];

Kt = lqe(At,eye(2),Ct,R1t,R2t) % Kalman gain

kalman2 = ss(At-Kt*Ct,Kt,eye(2),0);
Wheel_Kalman_D = c2d(kalman2,Ts,'foh')

% LQR based on linear model

A = [-3.1 58.4 62.7 0; 1 0 0 0; 40.1 -318 -766 0; 0 0 1 0];
B = [-148; 0; 1808; 0];
Q1 = diag([1 1 1 1]);
Q2 = 10;
L  = lqr(A,B,Q1,Q2)

% LQI with explicit integrator

Q1e = diag([1 1 1 1 0.1]);
Ctheta = [0 0 0 1];
sys = ss(A,B,Ctheta,0);
Le = lqi(sys,Q1e,Q2)
L = Le(1:4);
li = Le(5);

Feedback_Gain = Le(1:4);
Integral_Gain = Le(5);







