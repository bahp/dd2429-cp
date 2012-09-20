% function [norm_mat] = get_normalization_matrix(data);   
%
% Method: get all normalization matrices.  
%         It is: point_norm = norm_matrix * point. The norm_points 
%         have centroid 0 and average distance = sqrt(2))
% 
% Input: data (3*m,n) (not normalized) the data may have NaN values 
%        for m cameras and n points 
%        
% Output: norm_mat is a 3*m,3 matrix, which consists of all 
%         normalization matrices matrices, i.e. norm_mat(1:3,:) is the 
%         matrix for camera 1 
%

function [norm_mat] = get_normalization_matrix(data);   

% get Info 
am_cams = size(data,1)/3;  % amount of cameras.
am_points = size(data,2);  % amount of points. 

%-------------------------------------------------------------------------
%                         FILL IN THIS PART
%-------------------------------------------------------------------------
% Convert NaN to zeros.
data_NaN_Zero = data;
data_NaN_Zero(isnan(data_NaN_Zero)) = 0;

% Centroids.
centroids_points = reshape(sum(data_NaN_Zero,2),3,am_cams);
centroids = centroids_points(1:2,:)./repmat(centroids_points(3,:),2,1);
centroids = [centroids; ones(1,size(centroids,2))];

% Distance.
% 1 - Add centroids where NaNs are.
data_plus_centroid = data;
for i=1:am_cams
   for j=1:am_points
        if(isnan(data_plus_centroid(3*i-2,j)))
            data_plus_centroid(3*i-2:3*i,j)= centroids(:,i);
        end
   end
end

% 2 - Calculate distance.
distance_matrix = data_plus_centroid - ...
                  repmat(reshape(centroids,am_cams*3,1), 1, am_points);

distances = zeros(1,3);
for i=1:am_cams
    sum_row = sum(distance_matrix(i*3-2:i*3,:).^2,1);
    distances(i) = sum(sqrt(sum_row),2)./centroids_points(3,i);
end

% Create norm_mat.
for i=1:am_cams
    norm_mat(i*3-2:i*3,:)= ...
       [sqrt(2)/distances(i) 0 -sqrt(2)*centroids(1,i)/distances(i); ...
        0 sqrt(2)/distances(i) -sqrt(2)*centroids(2,i)/distances(i);...
        0 0 1];
end

