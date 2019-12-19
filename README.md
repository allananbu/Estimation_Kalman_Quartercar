# Estimation_Kalman_Quartercar
Estimating states of Quarter car system using Discrete Kalman filter

four states of the system are estimated with below initializations:
Q=10; %process noise covariance
R=10; %measurement noise covariance
u=[10;10]; %control input
P=eye(4); %initial covariance
w=10*randn(4,1); %process noise
v=6*randn(1); %measurement noise
X=[100;1000;100;10000]; %initial state
