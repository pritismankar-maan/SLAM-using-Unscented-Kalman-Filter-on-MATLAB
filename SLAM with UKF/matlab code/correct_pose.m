function [corr_current_pose,corr_cv,corr_landmark_obs,...
                                  cor_bet_pose_sensorid] = correct_pose(...
                                                    pred_current_pose,...
                                                    pred_cv,sensor,...
                                                    is_landmark_obs,...
                                                    cor_bet_pose_sensorid)
%   introduce sensor noise (variance = sqrt(0.05))
    Q = eye(2)*0.15;
    initial_cv_lmark = eye(2)*0.01;
% calculate the observed position of landmarks
for i=1:size(sensor,2)
    obs_data = [sensor(i).range;sensor(i).bearing];
% if the landmark is to be seen for the first time, initilize the
% landmark's position based on the observed location. Only when we have the
% same landmark's location from another timestep, then only we can predict
% and correct to the landmark's actual location on the map.
    lis_landmark_obs = is_landmark_obs([sensor(i).id]);
    if(strcmp(lis_landmark_obs.is_obs,'false'))
% ------------ landmark is being observed for the first time --------------  
        prev_len = length(pred_current_pose);
% append initial values to pose vector and covariance matrix for given
% landmark
        pred_current_pose = [pred_current_pose;obs_data(1);obs_data(2)];
        pred_cv = [pred_cv           zeros(prev_len,2);
                   zeros(2,prev_len) initial_cv_lmark];
% correlate the data from sensor with data stored in current_pose to plot
        cor_bet_pose_sensorid = [cor_bet_pose_sensorid;
                                 length(pred_current_pose)-1 sensor(i).id];       
    end
% get weights of mean and covariance as well as sigma points locations
% irrespective of whether or not the landmark has been observed
    [wght_mean,wght_cv,sigma_t] = get_wghts_sigma_pnts(pred_current_pose,...
                                                       pred_cv);
% normalize the angular pose of the robot    
     sigma_t(3,:) = normalize_angle(sigma_t(3,:)); 
     pred_current_pose_sig = sigma_t;
%   for each observed landmark, calculate it's location based on range
%   bearing sensor formula. Calculate the observed landmark data and
%   predicted landmark data and correct the predicted pose and covariance. 
%   data     = [range ; bearing]
%   location = [x ; y]
%   Note: If a landmark is observed for the first time, the observed data
%   and the predicted data should be the same.
    obs_data = [sensor(i).range;sensor(i).bearing]; 
    if(strcmp(lis_landmark_obs.is_obs,'false'))
% ------------ landmark is being observed for the first time --------------
    obs_loc_sig = [sigma_t(1,:) + ...
                   sigma_t(end-1,:).*cos(normalize_angle(sigma_t(end,:)+...
                                                sigma_t(3,:)));
                   sigma_t(2,:) + ...
                   sigma_t(end-1,:).*sin(normalize_angle(sigma_t(end,:)+...
                                                sigma_t(3,:)))];
    sigma_t(end-1,:) = obs_loc_sig(1,:);
    sigma_t(end,:) = obs_loc_sig(2,:);
% if the landmark is to be seen for the first time, initilize the
% landmark's position based on the observed location. Only when we have the
% same landmark's location from another timestep, then only we can predict
% and correct to the landmark's actual location on the map.
        pred_current_pose_sig(end-1,:) = obs_loc_sig(1,:);
        pred_current_pose_sig(end,:) = obs_loc_sig(2,:);
        pred_current_pose = sigma_t*transpose(wght_mean);
% Theta should be recovered by summing up the sines and cosines
        cosines = cos(sigma_t(3,:))*transpose(wght_mean);
        sines = sin(sigma_t(3,:))*transpose(wght_mean);
% recompute the angle and normalize it
        pred_current_pose(3) = normalize_angle(atan2(sines, cosines));                                          
% calculate updated cv based on new updated current pose similar to as done
% in prediction step
        diff_current_pose = sigma_t - pred_current_pose;
        diff_current_pose(3,:) = normalize_angle(diff_current_pose(3,:));
        pred_cv = (wght_cv.*diff_current_pose)*transpose(diff_current_pose);
        is_landmark_obs([sensor(i).id]).is_obs = 'true';
        continue;
    end
% -------------- landmark has been observed before --------------    
% find the index of the observed landmark in current pose vector
    index = find(cor_bet_pose_sensorid(:,2) == sensor(i).id);
    index = cor_bet_pose_sensorid(index,1);
% calculate the predicted location of ladmark's sigma points   
    err_loc_sig = [pred_current_pose_sig(index,:) - pred_current_pose_sig(1,:);
                   pred_current_pose_sig(index+1,:) - pred_current_pose_sig(2,:)];

% calculate certain values, to evalute Jacobian & predicted landmark's location
    q = err_loc_sig.*err_loc_sig;        
    q = q(1,:)+q(2,:);
    pred_data_sig    = [                    sqrt(q); 
                       normalize_angle(atan2(err_loc_sig(2,:),err_loc_sig(1,:)) ...
                                    - pred_current_pose(3))];
                                
% calculate the predicted landmark pose        
    pred_pose_lmark_x = wght_mean*transpose(cos(pred_data_sig(2,:)));
    pred_pose_lmark_y = wght_mean*transpose(sin(pred_data_sig(2,:)));
    pred_pose_lmark = [pred_data_sig(1,:)*transpose(wght_mean); 
                       atan2(pred_pose_lmark_y,pred_pose_lmark_x)];    

% difference between predicted landmark's sigma points and predicted 
% landmark's weighted position 
    diff_measurement = pred_data_sig-pred_pose_lmark;
    diff_measurement(2,:) = normalize_angle(diff_measurement(2,:));
% calculate St (metric for error percentage in observation data)
    St = wght_cv.*diff_measurement*transpose(diff_measurement)+Q;
% difference between predicted sigma points and predicted weighted position 
    diff_sig = sigma_t-pred_current_pose;
    diff_sig(3,:) = normalize_angle(diff_sig(3,:));
    cv_temp = repmat(wght_cv,length(pred_current_pose),1).*diff_sig*...
                                          transpose(diff_measurement);    
% calculte Kalman gain    
    Kt = cv_temp*inv(St);
    
%   correct robot pose,landmarks pose & covariance matrix with every iteration    
    pred_current_pose = pred_current_pose + Kt*normalize_diff(obs_data - ...
                                                              pred_pose_lmark);
    pred_current_pose(3) = normalize_angle(pred_current_pose(3));                                                       
    pred_cv = pred_cv - Kt*St*transpose(Kt);
   
end
%   assign the return variables
    corr_current_pose = pred_current_pose;
    corr_cv = pred_cv;
    corr_landmark_obs = is_landmark_obs;
end