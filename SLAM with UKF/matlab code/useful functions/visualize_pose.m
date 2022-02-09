function visualize_pose(current_pose,cv,landmarks,is_landmark_obs,sensor,...
                        start,cor_bet_pose_sensorid)
% In this function we are plotting the below things at each timestep:
%
% 1) Robot's estimated position by EKF Algorithm
% 2) Gaussian distribution(ellipse) around the Robot corrected position
% 3) Landmark's actual position
% 4) Landmark's estimated position by EKF Algorithm
% 5) Gaussian distribution(ellipse) around the Landmarks corrected position
% 6) Line joining the Robot and observed Landmark's estimated position. 
    clf;
    hold on
    grid("on")
    Lmarks = struct2cell(landmarks);
%   total number of angular points needed around the ellipse for plotting    
    elp_pnts = 100;elp_ang = linspace(0,2*pi,elp_pnts);
    elp_cos = cos(elp_ang);elp_sin = sin(elp_ang);
    elp_cos_sin =transpose([elp_cos(:) elp_sin(:)]);
% plot Landmark's actual position    
    plot(cell2mat(Lmarks(2,:)), cell2mat(Lmarks(3,:)), 'k+', 'markersize',...
                                              10, 'linewidth', 3); 
    
    for(i=1:length(is_landmark_obs))
        if(strcmp(is_landmark_obs(i).is_obs,'true'))
% find index of the current landmark in the pose vector             
        index = find(cor_bet_pose_sensorid(:,2) == i);
        index = cor_bet_pose_sensorid(index,1);
% for each observed landmark...
% plot Landmark's estimated position by EKF Algorithm            
        lmark_plot = plot(current_pose(index),current_pose(index+1),'+');
% display properties of landmark
        lmark_plot.Color = 'b';
        lmark_plot.LineWidth = 3;
% plot Gaussian distribution(ellipse) around the Landmarks corrected position                
        [vector,value] = eig(cv(index:index+1,index:index+1));
        d = sqrt(chi2inv(0.6,2))*sqrt(value);
        elp = (vector*d*elp_cos_sin) + repmat(current_pose(index:index+1),1,...
                                              size(elp_cos_sin,2));
        lmark_plot = plot(elp(1,:),elp(2,:));  
% display properties of landmark
        lmark_plot.Color = 'b';
        lmark_plot.LineWidth = 3;  
% plot Line joining the Robot and observed Landmark's estimated position.
        if(find([sensor.id]==i))
            lmark_x = current_pose(index);
            lmark_y = current_pose(index+1);
            line([current_pose(1), lmark_x],[current_pose(2), lmark_y],...
                 'color', 'k', 'linewidth', 1);
        end
        end
    end
% plot Robot's estimated position by EKF Algorithm
    robot_plot = plot(current_pose(1),current_pose(2),'s');
% display properties of robot
    robot_plot.Color = 'r';
    robot_plot.LineWidth = 1.5;
% plot Gaussian distribution(ellipse) around the Robot corrected position     
    [vector,value] = eig(cv(1:2,1:2));
    d = sqrt(chi2inv(0.6,2))*sqrt(value);
    elp = (vector*d*elp_cos_sin) + repmat(current_pose(1:2),1,...
                                          size(elp_cos_sin,2));
    robot_plot = plot(elp(1,:),elp(2,:));  
% display properties of robot
    robot_plot.Color = 'r';
    robot_plot.LineWidth = 1.5;    
    xlabel('x position'),ylabel('y position');
    legend('Actual Lmark pos','Estimated Lmark pos','Covariance with Est. pos');
    xlim([-2, 12])
    ylim([-2, 12])
    hold off
    drawnow;
    
% Capture the plot as an image 
    frame = getframe; 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    filename = 'SLAM with UKF.gif';
    if start == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end    
end