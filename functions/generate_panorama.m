% Script: generate_panorama
%
% Method: Generate one image out of multiple images. 
%         Where all images are from a camera with the 
%         same(!) center of projection. All the images 
%         are registered to one reference view (ref_view)
%

% adjustments
format compact;
format short g;

% Parameters fot the process 
ref_view = 3; % Reference view (third)
am_cams = 3;  % Amount of cameras
name_file_images = 'names_images_kthsmall.txt';
name_panorama = '../images/panorama_image.jpg';

% initialise
homographies = cell(am_cams,1); % we have: point(ref_view) = homographies{i} * point(image i)
data = [];
data_norm = [];

% load the images 
[images, name_loaded_images] = load_images_grey(name_file_images, am_cams);

% click some points or load the data 
load '../data/data_kth_7.mat'; % if you load a clicked sequence 
%data = click_multi_view(images, am_cams, data, 0); % for clicking and displaying data
%save '../data/data_kth_5.mat' data; % for later use 

% normalize the data 
[norm_mat] = get_normalization_matrices(data);
for hi1 = 1:am_cams
  data_norm(hi1*3-2:hi1*3,:) = norm_mat(hi1*3-2:hi1*3,:) * data(hi1*3-2:hi1*3,:); 
end

% determine all homographies to a reference view
%-------------------------------------------------------------------------
%                           FILL IN THIS PART
%
%        Remember, you have to set homographies{ref_view} as well
%-------------------------------------------------------------------------
       
% Convert NaN to zeros.
data_NaN_Zero = data;
data_NaN_Zero(isnan(data_NaN_Zero)) = 0;

% Number of points selected in each image(avoid problem accesing lines 79/105).
points_each_image = zeros(1,am_cams+1);
for i=1:am_cams
   points_each_image(i) = sum(data_NaN_Zero(i*3,:),2);
end

% Calculate homographies with or without normalization.
NORM = 1;
if (NORM)
    % NORMALIZED
    display('Normalized choosed')
    start = 1;
    finish = points_each_image(1);
    for i=1:am_cams
        if i == ref_view
            homographies{ref_view} = eye(3);
        else
            H_NORM = ...
               det_homographies( ...
                  data_norm(i*3-2:i*3,start:finish), ...
                  data_norm(am_cams*3-2:am_cams*3,start:finish));
            homographies{i} = ...
               norm_mat(am_cams*3-2:am_cams*3,1:3)^-1* ...
               H_NORM*norm_mat(i*3-2:i*3,1:3); 
        end
        
        % update values.
        start = finish + 1;
        finish = finish + points_each_image(i+1);
    end
   
    %H_NORM1 = det_homographies(data_norm(1:3,1:4),data_norm(7:9,1:4));
    %H1 = (norm_mat(7:9,1:3))^-1*H_NORM1*norm_mat(1:3,1:3);
    %homographies{1} = H1;
    %H_NORM2 = det_homographies(data_norm(4:6,5:8),data_norm(7:9,5:8));
    %H2 = (norm_mat(7:9,1:3))^-1*H_NORM2*norm_mat(4:6,1:3);
    %homographies{2} = H2;
    %homographies{ref_view} = eye(3);
else
    % NOT NORMALIZED
    display('NOT Normalized choosed'); 
    start = 1;
    finish = points_each_image(1);
    for i=1:am_cams
        if i == ref_view
            homographies{ref_view} = eye(3);
        else
            homographies{i} = ...
               det_homographies( ...
                  data(i*3-2:i*3,start:finish), ...
                  data(am_cams*3-2:am_cams*3,start:finish));
        end
        
        % update values.  
        start = finish + 1;
        finish = finish + points_each_image(i+1);
    end
    
    %homographies{1} = det_homographies(data(1:4,1:4),data(7:9,1:4));
    %homographies{2} = det_homographies(data(4:6,5:8),data(7:9,5:8));
    %homographies{ref_view} = eye(3);
end

%-------------------------------------------------------------------------

% check error in the estimated homographies
for hi1 = 1:am_cams
  [error_mean, error_max] = check_error_homographies(homographies{hi1},data(3*hi1-2:3*hi1,:),data(3*ref_view-2:3*ref_view,:));
  fprintf('Between view %d and ref. view; ', hi1); % Info
  fprintf('average error: %5.2f; maximum error: %5.2f \n', error_mean, error_max); 
end

% create the new warped image
panorama_image = generate_warped_image(images, homographies);

% show it
figure;  
show_image_grey(panorama_image);

% save it 
save_image_grey(name_panorama,panorama_image);
