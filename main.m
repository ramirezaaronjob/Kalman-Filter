 
%Initialization
clc, close all
clear
 
%Load data and correct IMU initial offset
[time, data] = rtpload('EKF_DATA_circle.txt'); %data of the circle in front of Engineering Building
%[time, data] = rtpload('EKF_DATA_Rutgers_ParkingLot.txt'); %data of the circle in front of Engineering Building
 
%Get Odometry IMU and GPS data (x, y, theta, covariance)
Odom_x = data.O_x;
Odom_y = data.O_y;
Odom_theta = data.O_t;
 
Gps_x = data.G_x;
Gps_y = data.G_y;
 
Gps_Co_x = data.Co_gps_x;
Gps_Co_y = data.Co_gps_y;
 
IMU_heading = data.I_t;
IMU_Co_heading = data.Co_I_t;
 
%----------------------------------------------
 
% % Add noise to GPS
 noise_mean = 0.5;
 noise_std = 0.1;
 Gps_noise = noise_std .* randn(length(Odom_x), 2)+ noise_mean.*ones(length(Odom_x), 2);
% 
 Gps_x = data.G_x +Gps_noise(:,1);
 Gps_y = data.G_y + Gps_noise(:,2);
% 
 Gps_Co_x = data.Co_gps_x +Gps_noise(:,1);
 Gps_Co_y = data.Co_gps_y +Gps_noise(:,2);

%----------------------------------------------
 
% % Add noise to Odometry
 noise_mean = 0.5;
 noise_std = 0.1;
 Odom_noise = noise_std .* randn(length(Odom_x), 2)+ noise_mean.*ones(length(Odom_x), 2);
% 
 Odom_x = Odom_x + Odom_noise(:,1);
 Odom_y = Odom_y + Odom_noise(:,2);
 
 
%---------------------------------------------
 
% %Add noise to IMU covariance
 noise_mean = 0.5;
 noise_std = 0.1;
 IMU_noise = noise_std .* randn(length(Odom_x), 2)+ noise_mean.*ones(length(Odom_x), 2);
 IMU_Co_heading = IMU_Co_heading + IMU_noise(:, 1);
 
%---------------------------------------------
 
% Calibrate IMU to match with the robot's heading initially
IMU_heading = IMU_heading +(0.32981-0.237156)*ones(length(IMU_heading),1); 
%For EKF_DATA3
 
% figure(1), plot(time, data.O_t)
% figure(2), plot(time, data.I_t)
% figure(3), plot(data.O_x, data.O_y, 'x')
% figure(4), plot(time, IMU_heading)
 
%Velocity of the robot
V = 0.44;%0.083;
 
%Distance between 2 wheel
L = 1; %meter
 
%Angular Velocity
Omega = V*tan(Odom_theta(1))/L;
 
%set time_step
delta_t = 0.001; %0.001
 
%total=1:delta_t:length(Odom_x);
total=1:length(Odom_x);
 
%********INITIALIZE STATES***********
s.x = [Odom_x(1); Odom_y(1); V; Odom_theta(1); Omega]; %Enter State (1x5)
 
%Enter transistion matrix A (5x5)
s.A = [1 0 delta_t*cos(Odom_theta(1)) 0 0;
       0 1 delta_t*sin(Odom_theta(1)) 0 0;
       0 0 1                          0 0;
       0 0 0                          1 delta_t;
       0 0 0                          0 1]; 
 
%Define a process noise (stdev) of state: (Student can play with this number)
%Enter covariance matrix Q (5x5) for state x
 
s.Q = [.0004  0   0   0   0; %For EKF_DATA_circle
        0  .0004  0   0   0;
        0   0   .001  0   0;
        0   0    0  .001  0;
        0   0    0   0  .001]; 
 
%   s.Q = [.000000004  0   0   0   0; %For EKF_DATA_Rutgers_ParkingLot
%         0  .000000004  0   0   0;
%         0   0   .001  0   0;
%         0   0    0  .001  0;
%         0   0    0   0  .001];  
%Define the measurement matricx H:
%Enter measurement matrix H (5x5) for measurement model z
s.H = [ 1   0   0   0   0;
        0   1   0   0   0;
        0   0   1   0   0;
        0   0   0   1   0;
        0   0   0   0   1]; 
 
%Define a measurement error (stdev)
%Enter covariance matrix R (5x5) for measurement model z
s.R = [.04  0   0   0   0;
        0  .04  0   0   0;
        0   0  .01  0   0;
        0   0   0   0.01  0;
        0   0   0   0  .01]; 
%B matrix initialization:
s.B = [ 1   0   0   0   0;
        0   1   0   0   0;
        0   0   1   0   0;
        0   0   0   1   0;
        0   0   0   0   1];
%Enter initial value of u (5x5)    
s.u = [0; 0; 0; 0; 0];
 
%Enter initial covariance matrix P (5x5)
s.P = [.001  0   0   0   0;
        0  .001  0   0   0;
        0   0  .001  0   0;
        0   0   0  .001  0;
        0   0   0   0  .001]; 
 
%*********STORE DATA FOR PLOT***********    
true=[]; % truth voltage
X1=[];
X2=[];
X_heading=[];
 
%********START KALMAN FILTER***********

for t=1:length(total)
  s(t).A = [1 0 delta_t*cos(Odom_theta(t)) 0 0;
       0 1 delta_t*sin(Odom_theta(t)) 0 0;
       0 0 1                          0 0;
       0 0 0                          1 delta_t;
       0 0 0                          0 1]; %Enter transistion matrix A (5x5)
   s(t).A = [1 0 delta_t*cos(IMU_heading(t)) 0 0;
       0 1 delta_t*sin(IMU_heading(t)) 0 0;
       0 0 1                          0 0;
       0 0 0                          1 delta_t;
       0 0 0                          0 1]; %Enter transistion matrix A (5x5)
 
  s(t).R = [Gps_Co_x(t)  0             0   0                    0;
            0            Gps_Co_y(t)   0   0                    0;
            0            0            .01  0                    0;
            0            0             0   IMU_Co_heading(t)    0;
            0            0             0   0                   .01]; %Enter covariance matrix R (5x5) for measurement model z
 
%    
   %s(end).z =   [Odom_x(t); Odom_y(t); V; Odom_theta(t); Omega]; %Enter State (1x5); % create a measurement
   %s(t).z =   [Odom_x(t); Odom_y(t); V; IMU_heading(t); Omega]; %Enter State (1x5); % create a measurement
   s(t).z =   [Gps_x(t); Gps_y(t); V; IMU_heading(t); Omega]; %Enter State (1x5); % create a measurement with adding GPS data
   s(t+1)=Kalman_Filter(s(t)); % perform a Kalman filter iteration
% All for you is to write a “Kalman Filter”
 
   %For Plot only
   
   X = s(t).x;
   X1(t,:) = X(1,:);
   X2(t,:) = X(2,:);
   
   X_theta = s(t).x;
   X_heading(t,:) = X_theta(4,:);
   
% % %    %pause(1);
%      hold on
%      grid on
% % plot Odometry(x,y) data:
%      plot(Odom_x(t), Odom_y(t), 'r');
%      % plot Odometry(x,y) data:
%      hold on
%      plot(Gps_x(t), Gps_y(t), 'k');
% % plot KF estimates:
%      hold on
%      plot(X1(t), X2(t),'b');
%      %legend([hz hgps hk],'Odometry','gps calibrated','Kalman output', 0)
%      %title('Fusion of GPS+IMU and ODOMETRY in Position')
%      M(t) = getframe;
%      hold off
end
%%
% *************Plot the Position***********
    figure(1)
    hold on
    grid on
% plot Odometry(x,y) data:
     hz = plot(Odom_x, Odom_y, '.r');
     % plot Odometry(x,y) data:
     hgps = plot(Gps_x, Gps_y, '.k');
% plot KF estimates:
     hk=plot(X1, X2,'.b');
     legend([hz hgps hk],'Odometry','gps calibrated','Kalman output', '0')
     title('Fusion of GPS+IMU and ODOMETRY in Position')
 
%*******Plot Heading*********
    figure(2)
    hold on
    grid on
    odom_heading=plot(time, data.O_t, 'r');
    imu_heading=plot(time, IMU_heading, 'k');
    KF_heading = plot(X_heading, 'b');
    legend([odom_heading, imu_heading, KF_heading],'Odometry heading','IMU heading', 'KF heading','0')
    title('Fusion of GPS+IMU and ODOMETRY in heading')
 
function s = Kalman_Filter(s)
  
   s.x = s.A*s.x + s.B*s.u; %project state ahead
   s.P = s.A * s.P * s.A' + s.Q; %Project error covariance
   K = s.P*s.H'*inv(s.H*s.P*s.H'+s.R); %compute kalman gain
   s.x = s.x + K*(s.z-s.H*s.x); %update estimate
   s.P = s.P - K*s.H*s.P;%update error covariance
end