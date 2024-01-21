% (c) 2024 Gianni Cassoni <giannicassoni@polimi.it>
% This code is licensed under MIT license 
function [x] = EKF(x, d, u, t, FIR_dim, x0, P0, Q_noise, R_noise)
% This function implements the extended Kalman filter (EKF) for a nonlinear system
% Inputs:
% x: state vector
% d: measurement vector
% u: input vector
% t: time step
% FIR_dim: dimension of the state vector
% x0: initial state vector
% P0: initial state covariance matrix
% Q_noise: process noise covariance matrix
% R_noise: measurement noise covariance matrix
% Outputs:
% x: updated state vector

persistent P dim Q R % declare persistent variables to store previous values
% The following is initialization, and is executed once
if (ischar(x) && strcmp(x,'initial')) % check if the input x is a string 'initial'
    dim =  FIR_dim; % set the dimension of the state vector
    P = P0; % set the initial state covariance matrix
    Q = Q_noise; % set the process noise covariance matrix
    R = R_noise; % set the measurement noise covariance matrix
    x = x0'; % set the initial state vector
end

[x,F,~,~] = BDFT_Discrete(x,u,t); % predict the state and the state transition matrix using the BDFT_Discrete function
P = F*P*F' + Q; % predict the state covariance matrix
[~,~,h,H] = BDFT_Discrete(x,u,t); % compute the measurement and the measurement matrix using the BDFT_Discrete function
r = d - h; % compute the measurement residual
S = H*P*H'+ R; % compute the residual covariance matrix
K = P*H'/S; % compute the Kalman gain
x = x + K*r; % update the state vector
P = (eye(dim) - K*H)*P; % update the state covariance matrix

end


