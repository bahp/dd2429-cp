% function [cams, cam_centers] = det_uncalib_stereo_cameras(F); 
%
% Method: Calculate the first and second uncalibrated camera matrix
%         from the F-matrix. 
% 
% Input:  F - Fundamental matrix with the last singular value 0 
% 
% Output: cams (6,4), where cams(1:3,:) is the first and 
%                     cams(4:6,:) is the second camera      
%         cam_centers(4,2) where (:,1) is the first and (:,2) 
%                          the second camera center
%

function [cams, cam_centers] = det_uncalib_stereo_cameras(F)

%-------------------------------------------------------------------------
%                           FILL IN THIS PART
%-------------------------------------------------------------------------
% Calculate cameras a matrices Ma = (I|0) and Mb = (SF|h).
% Calculate Ma.
Ma = [eye(3) zeros(3,1)];  % Calculate Ma.

% Calculate Mb.
[U,S,V] = svd(F');             % Calculate F'h = 0 (epipol 2nd camera).
h = V(:,end);
S = [ 0 -1 1; 1 0 -1; -1 1 0]; % Calculate S (antisymmetric matrix).
Mb = [S*F h];

% Calculate camera centers.
% Camera A center (Ma*ta = [0,0,0]');
[U,S,V] = svd(Ma);
ta = V(:,end);

% Camera B center (Mb*tb = [0,0,0]');
[U,S,V] = svd(Mb);
tb = V(:,end);

% Set the return values.
cams = [Ma; Mb];
cam_centers = [ta tb];