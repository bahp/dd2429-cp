% function [error_average, error_max] = check_reprojection_error(data, cam, model)
%
% Method: Evaluates average and maximum error 
%         between the reprojected image points (cam*model) and the 
%         given image points (data), i.e. data = cam * model 
%
% Input:  data (3*m,n) matrix with m cameras and n points 
%         cam (3*m,4) matrix 
%         model (4,n) matrix
%       
% Output: the average error (error_average) and maximum error (error_max)
%      

function [error_average, error_max] = check_reprojection_error(data, cam, model)

%-------------------------------------------------------------------------
%                          FILL IN THIS PART
%-------------------------------------------------------------------------
% Initialize variables.
n_points = size(model,2);
n_cams = size(cam,1)/3;
v_errors = zeros(1,n_cams*n_points);

% Calculate the re-projected data (3*m,n) as data.
reprojected_data = cam*model;

% Normalize points (we are working with cartesian [x y z 1].
for i = 1:n_cams
   reprojected_data(3*i-2:3*i,:) = ...
        norm_points_to_one(reprojected_data(3*i-2:3*i,:));
end

% Fill the vector with errors.
pos = 1;
for i = 1:n_cams
    for j = 1:n_points
        current_error = reprojected_data(3*i-2:3*i,j) - data(3*i-2:3*i,j);
        v_errors(pos) = sqrt(current_error'*current_error);
        pos = pos + 1;
    end
end

% Returned values
error_average = sum(v_errors)/(n_points*n_cams);
error_max = max(v_errors);