% Test the functions of lab1.
clear; clc;

display('-----------------------------------------------')
display('             TEST ASSIGNMENT 2                 ')
display('-----------------------------------------------')

% --------------------------------------------------
% Test 1 - Essential matrix.
% --------------------------------------------------
% Load data.
clear; load('test_det_e_mat.mat')                 

% Display.
disp(' 1 - Testing Essential matrix ...')
disp('    Debug E:')
disp(E)

display('    My calculated E:')
my_E = det_E_matrix(d1,d2,K1,K2);
disp(my_E)

% --------------------------------------------------
% Test 2 - Determining the model by triangulation.
% --------------------------------------------------
% Load data.
clear; load('test_det_model.mat')

% Display:
disp('2 - Testing the model ...')
disp(sprintf('    Debug Model (sum: %d)',sum(sum(model))))
disp(model(:,1:3))

my_model = det_model(cams_norm,data_norm);
disp(sprintf('    My Calculated Model (sum: %d)',sum(sum(my_model))))
disp(my_model(:,1:3))

% --------------------------------------------------
% Test 3 - Determining stereo camera.
% --------------------------------------------------
% Load data.
clear; load('test_det_stereo_cam');

% Display.
disp('3 - Testing the stereo cam ...');
disp('    Debug cams:');
disp(cams)
disp('    Debug cam_centers:');
disp(cam_centers);

[my_cams, my_cam_centers] = det_stereo_cameras(E,K1,K2,pts);
disp('    My calculated cams:');
disp(my_cams);
disp('    My calculated cam_centers:');
disp(my_cam_centers);

% --------------------------------------------------
% Test 4 - Determining re-projection error.
% --------------------------------------------------
% Load data.
clear; load('test_reproj_err');

% Display:
disp('4 - Re-projection error ...')
disp(sprintf('    Debug Mean Error: %s',error_average));
disp(sprintf('    Debug Max Error : %s',error_max));

[my_error_average, my_error_max] = check_reprojection_error(data, cams, model);
disp(sprintf('    My calculated Mean Error: %s',my_error_average));
disp(sprintf('    My calculated Max Error : %s',my_error_max));

% --------------------------------------------------
% Test 5 - Triangulation.
% --------------------------------------------------
% Load data.
clear; load('test_delaunay');

% Display:
disp('5 - Delaunay triangulaton ...')
disp(sprintf('    Debug triangulation (sum %d):',sum(sum(triang))));

[my_triang] = get_delaunay_triang(data, image_ref);
disp(sprintf('    My calculated triangulation (sum %d):',sum(sum(my_triang))));

disp(sprintf('    Difference should be 0 : %d',sum(sum(triang ~= my_triang))));