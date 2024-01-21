# EKF
Extended kalman filter
example:  
```matlab
% Initialize the EKF with the first two measurements and the input 
x = EKF('initial',[d(2) d(3)]',u(1), t(1), dim, x0, P0, Q, R); 

% Preallocate a matrix to store the state estimates 
x_ekf = zeros(length(d),dim);

% Run the EKF for each measurement 
for j = 2:length(d) 
    x = EKF(x,[d(j+1) d(j+2)]',u(j), t(j)); 
    x_ekf(j, :) = x'; 
end
```
