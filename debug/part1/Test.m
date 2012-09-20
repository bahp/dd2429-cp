% Test the two functions of lab2.
clear; clc; close all;

load('test_det_h.mat')
load('test_norm_mat.mat')

display('-----------------------------------------------')
display('             TEST ASSIGNMENT 1                 ')
display('-----------------------------------------------')

% Test 1 - Homographies.
display('Testing homographies ...')
display('Debug H:')
H

display('My calculated H:')
det_homographies(p1,p2)

display( '         ------------------------             ')

% Test 2 - Normalization.
display('Testing Normalization ...')
display('Debug norm_mat:')
norm_mat

display('My calculated  norm_mat')
get_normalization_matrices(data)