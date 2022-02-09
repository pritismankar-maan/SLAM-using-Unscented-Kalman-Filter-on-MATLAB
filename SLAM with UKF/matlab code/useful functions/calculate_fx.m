function Fx = calculate_fx(len)
total_obs_landmarks = (len-3)/2;
Fx_robot_pose = eye(3);
if total_obs_landmarks == 0
    Fx = [Fx_robot_pose];
else    
    Fx_map_pose = zeros(3,2*total_obs_landmarks);
    Fx = [Fx_robot_pose Fx_map_pose];
end
end
