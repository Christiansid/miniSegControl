% This file is automatically run when 'lab3.slx' is opened

Ts = 0.025;   % Simulink model time-step
set_param('lab3', 'FixedStep', num2str(Ts));

% Set default (zero) values for controller variables

IMU_Kalman_D = ss(zeros(2),zeros(2),zeros(2),0,Ts);     % Kalman filter 1
Wheel_Kalman_D = ss(zeros(2),zeros(2,1),zeros(2),0,Ts); % Kalman filter 2
Feedback_Gain = [0 0 0 0];                              % State feedback gain
Integral_Gain = 0;                                      % Integral gain
