function [pred_current_pose,pred_cv] = predict_pose(current_pose,cv,...
                                                    odometry)

% get weights of mean, covariance and sigma points points locations at 
% previous timestep
[wght_mean,wght_cv,sigma_tminus1] = get_wghts_sigma_pnts(current_pose,...
                                                         cv);
% calculat length of current poses (robot + observed landmarks)
len = length(current_pose);
% calculate 'Fx' matrix (mapping matrix) to predict the updated robot's and 
% landmark poses based on input odometry command, time & environment. Here,
% the landmarks are assumed to be still and hence 'Fx_map_pose' matrix is a
% zero matrix.
Fx = calculate_fx(len);
% calculate odometry matrix to predict new pose of all sigma points. Since, 
% the input from odometry/controls can only manipulate the robot(not 
% landmarks) pose ---
% current_pose(3): robot's previous angular value (theta)
odometry_matrix =  [odometry.t.*(cos(normalize_angle(sigma_tminus1(3,:)+odometry.r1)));
                   odometry.t.*(sin(normalize_angle(sigma_tminus1(3,:)+odometry.r1)));
                   repmat(normalize_angle(odometry.r1+odometry.r2),1,2*len+1)];

% calculate location of sigma points based on odometry data
sigma_t = sigma_tminus1+transpose(Fx)*odometry_matrix;
% get new pose as the weighted mean of sigma points
new_pose = transpose(wght_mean*transpose(sigma_t));
% calculate total x component and y component
curr_pose_x = wght_mean*transpose(cos(sigma_t(3,:)));
curr_pose_y = wght_mean*transpose(sin(sigma_t(3,:)));
% always normalize agnle
theta = normalize_angle(atan2(curr_pose_y,curr_pose_x));
new_pose(3) = theta; 
% Odometry noise
Rt = [0.1   0     0;
       0   0.1    0;
       0    0     0];
% calculate new predicted cv
diff = sigma_t-new_pose;
diff(3,:) = normalize_angle(diff(3,:));
new_cv = wght_cv.*diff*transpose(diff);
% add motion noise
new_cv(1:3,1:3) = new_cv(1:3,1:3) + Rt; 
% assign return variables
pred_cv = new_cv;
pred_current_pose = new_pose;

end