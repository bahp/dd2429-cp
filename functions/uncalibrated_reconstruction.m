% Script: uncalibrated_reconstruction.m 
%
% Method: Performs a reconstruction of two 
%         uncalibrated cameras. The Fundametal matrix determines 
%         uniquely both cameras. The point-structure is obtained 
%         by triangulation. The projective reconstruction is 
%         rectified to a metric reconstruction with knowledge 
%         about the 3D object.
%         Finally the result is stored as a VRML model 
% 

%----------------------------------------
% Initialisation
%----------------------------------------

% Parameters
image_ref = 1; % Reference view
am_cams = 2; % amount of cameras
name_file_images = 'names_images_toyhouse.txt';
image_name = 'toyhouse1.jpg'; 
name_file_vrml = '../vrml/vrml_model.wrl';
flag_synthetic_data = 0; % 0 - we work with real data; 1- we work with syntetic data

% adjustments
format compact;
format short g;

% initialise
data = [];
data_norm = [];
cams = zeros(6,4);
cams_norm = zeros(6,4);
cam_centers = zeros(4,2); 
model_synthetic = [];  

%----------------------------------------
% Get data: synthetic & real
%----------------------------------------

% load synthetic data: 
% data_sim, cam_sim, model_sim; with  data_sim = cam_sim * model_sim;
if (flag_synthetic_data)
  load 'sphere_data.mat';
  data = data_sim;
end

% real data 
if (~flag_synthetic_data)
  load '../data/toyhouse_data_72.mat';
  %data = [];
  [images,image_names] = load_images_grey(name_file_images, am_cams)
  %data = click_multi_view(images, am_cams, data, 2); % for clicking and displaying data
  %save '../data/toyhouse_data.mat' data;
end

%-----------------------------------------------
% Get the Fundamental matrix & Cameras & model
%-----------------------------------------------

% determine the Fundamental matrix 
F = det_F_matrix(data(1:3,:), data(4:6,:),1);

% get the projective reconstruction 
[cams, cam_centers] = det_uncalib_stereo_cameras(F); 

% obtain the complete model by triangulation
% first normalize the model and the cameras 
[norm_mat] = get_normalization_matrices(data);
data_norm(1:3,:) = norm_mat(1:3,:) * data(1:3,:);
data_norm(4:6,:) = norm_mat(4:6,:) * data(4:6,:);
cams_norm(1:3,:) = norm_mat(1:3,:) * cams(1:3,:);
cams_norm(4:6,:) = norm_mat(4:6,:) * cams(4:6,:);
model = det_model(cams_norm, data_norm);

% check the reprojection error 
[error_average, error_max] = check_reprojection_error(data, cams, model);
fprintf('\n\nThe reprojection error: data = cams * model is: \n');
fprintf('Average error: %5.2fpixel; Maximum error: %5.2fpixel \n', error_average, error_max); 

%-----------------------------------------------------
% Rectify the reconstruction: cameras and 3D model
%-----------------------------------------------------
if (flag_synthetic_data)
    
   % Print model and model_sim. 
   model(:,1:5)
   model_sim(:,1:5)
    
   model_synthetic_points = [1,13,25,37,48];
   model_synthetic = [];  
   for hi1 = 1:5
      hd1 = model_synthetic_points(1,hi1); 
      model_synthetic = [model_synthetic, model_sim(:,hd1)];
   end
  
else 
  
  %-----------------------------------------------------------------------
  %                         FILL IN THIS PART 
  %-----------------------------------------------------------------------
  p1 = [0 0 0 1]';            % 1 in pictures.
  p2 = p1 +  [0  10  0  0]';  % 2 in pictures.
  p3 = p1 +  [0   0  9  0]';  % 3 in pictures.
  p4 = p1 +  [10  5 15  0]';  % 4 in pictures.
  p5 = p1 +  [27  0  0  0]';  % 5 in pictures.
  p6 = p1 +  [0  10  9  0]';  % 6 in pictures.
  p7 = p1 +  [10  0  9  0]';  % 7 in pictures.
  p11 = p1 + [27  5 15  0]';  % 11 in pictures.
  model_sim = [p1 p2 p3 p5 p11];
  
  % Print model and model_sim.
  model(:,1:5)
  model_sim(:,1:5)
  
  figure(4);
  plot3(model_sim(1,:),model_sim(2,:),model_sim(3,:),'xb')
  axis([-1 30 -1 15 0 20])
  xlabel('x'); ylabel('y'); zlabel('z');
  
  model_synthetic_points = [1,2,3,5,11];
  model_synthetic = [];  
  for hi1 = 1:length(model_synthetic_points)
     hd1 = model_synthetic_points(1,hi1); 
     model_synthetic = [model_synthetic, model_sim(:,hi1)];
  end
end

% get the rectification matrix 
H = det_rectification_matrix(model(:,model_synthetic_points), model_synthetic); 

% rectify the cameras and the model
%-------------------------------------------------------------------------
%                           FILL IN THIS PART
%-------------------------------------------------------------------------
model_rec = H*model;              % rectified model (P'=HP).
cams_rec = cams*inv(H);           % rectified cams matrices. (M'=Minv(H))
cam_centers_rec = H*cam_centers;  % rectified cam centers.

% normalize the the 4th coordinate of model points is 1
[model_rec] = norm_points_to_one(model_rec);  
 
% normalize the 4th coordinate of the camera centers
[cam_centers_rec] = norm_points_to_one(cam_centers_rec);    


%---------------------------------------------
% Triangulation & Visualization & VRML model
%---------------------------------------------

triang = get_delaunay_triang(data, image_ref);
% triang = [];

% visualize the reconstruction 
visualize(model_rec, cam_centers_rec, triang, 1); 

% save the vrml model + texture
if (~flag_synthetic_data)
  image_size = [size(images{1},1) size(images{1},2)];
  save_vrml(data, model_rec, triang, name_file_vrml, image_name, image_size, 0, image_ref); 
end
