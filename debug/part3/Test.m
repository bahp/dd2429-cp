% Test the functions of lab3.
clear; clc; close all;

display('-----------------------------------------------')
display('             TEST ASSIGNMENT 3                 ')
display('-----------------------------------------------')

% --------------------------------------------------
% Test 1 - Fundamental matrix.
% --------------------------------------------------
% Load data.
clear; load('test_det_f_mat.mat')                 

% Display.
disp(' 1 - Testing Fundamental matrix ...')
disp('    Debug F:')
disp(F)

display('    My calculated F:')
my_F = det_F_matrix(d1,d2,0);
disp(my_F)

display('    Relation between normalized and not')
my_F2 = det_F_matrix(d1,d2,1);
disp(my_F ./ my_F2)


% --------------------------------------------------
% Test 2 - Determining the cameras and mdoel.
% --------------------------------------------------
% Load data.
clear; load('test_det_uncalib_stereo_cam.mat');

% Display.
disp(' 2 - Testing the stereo cam ...');
disp('    Debug cams:');
disp(cams)
disp('    Debug cam_centers:');
disp(cam_centers);

[my_cams, my_cam_centers] = det_uncalib_stereo_cameras(F);
disp('    My calculated cams:');
disp(my_cams);
disp('    My calculated cam_centers:');
disp(my_cam_centers);

% --------------------------------------------------
% Test 3 - Determining H (rectification matrix).
% --------------------------------------------------
clear; load('test_det_rect_mat.mat');

% Display.
disp('3 - Testing rectification matrix ...');
disp('    H:');
disp(H)

my_H = det_rectification_matrix(m1,m2);
disp('    My calculated H:');
disp(my_H);
