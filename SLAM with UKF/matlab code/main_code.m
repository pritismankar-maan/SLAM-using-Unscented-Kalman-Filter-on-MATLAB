%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        SLAM using Unscented Kalman Filter
%               Name - Pritisman Kar
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;close all;clc;

% read landmark as a structure from input text file 
[landmarks,is_landmark_obs] = read_lankmarks('C:\Users\HP\Documents\MATLAB\SLAM with UKF\sensor_data\world.txt');
% read sensor data as a structure from input text file 
sensor_data = read_sensor_data('C:\Users\HP\Documents\MATLAB\SLAM with UKF\sensor_data\sensor_data.txt');

% Intialization at t = 0
% ----------------------
% count the number of landmarks so as to set the size of be pose vector
% and covariance matrix. In reality, this vector and matrix should grow based
% on the new landmakrs observed by the sensor.
count_landmarks = size(landmarks,2);
% initial pose of the robot and the location of landmarks
current_pose = zeros(3,1);
% initial covariance matrix setup
cv = eye(3,3)*0.001;
% initial map setup -- Correletion between landmarks index from sensor 
% reading and final pose vector index
cor_bet_pose_sensorid = [];

% Run algorithm (t > 0)
% ----------------------   

for i=1:size(sensor_data.timestep,2)
%   perform prediction step to get predict pose and covariance matrix
    [pred_current_pose,pred_cv] = predict_pose(current_pose,cv,...
                                     sensor_data.timestep(i).odometry);
%   perform correction step to get corrected pose and covariance matrix
%   based on observation data
    [corr_current_pose,corr_cv,corr_landmark_obs,cor_bet_pose_sensorid]...
                                            = correct_pose(...
                                            pred_current_pose,pred_cv,...
                                            sensor_data.timestep(i).sensor,...
                                            is_landmark_obs,...
                                            cor_bet_pose_sensorid);
%   update the corrected pose as the current pose to be used in the next
%   iteration
    current_pose = corr_current_pose;
    cv= corr_cv;
    is_landmark_obs = corr_landmark_obs;    
%   display plot with each timestep    
    visualize_pose(current_pose,cv,landmarks,is_landmark_obs,...
                   sensor_data.timestep(i).sensor,i,cor_bet_pose_sensorid);
end