function [wght_mean,wght_cv,sigma_tminus1] = get_wghts_sigma_pnts(current_pose,...
                                                                  cv)                                                    
% get sigma points based on current pose and covariance 
sigma_tminus1 = compute_sigma_points(current_pose,cv);
% Recover mu and sigma
scale =3.0;
n = length(current_pose);
lambda = scale - n;
w0 = lambda/scale;
wght_mean = [w0, repmat(1/(2*scale),1,2*n)];
wght_cv = wght_mean;
end
