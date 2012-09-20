% function E = det_E_matrix(points1, points2, K1, K2);
%
% Method: Calculate the E matrix between two views from
%         point correspondences: points2^T * E * points1= 0
%         we use the normalize 8-point algorithm and 
%         enforce the constraint that the three singular 
%         values are: a,a,0. The data will be normalized here. 
%         Finally we will check how good the epipolar constraints:
%         points2^T * E * points1 = 0 are fullfilled.
% 
% Input:  points1, points2 of size (3,n) 
%         K1 is the internal camera matrix of the first camera; (3,3) matrix
%         K2 is the internal camera matrix of the second camera; (3,3) matrix
% 
% Output: E (3,3) with the singular values (a,a,0) 
% 

function E = det_E_matrix(pA, pB, Ka, Kb)

%-------------------------------------------------------------------------
%                         FILL IN THIS PART
%-------------------------------------------------------------------------

% Points in a normalized camera coordinate system.
pA_cam = (Ka)^-1*pA;
pB_cam = (Kb)^-1*pB;

% Points normalized.
[norm_matA] = get_normalization_matrices(pA_cam);
pA_cam_norm = norm_matA*pA_cam;

[norm_matB] = get_normalization_matrices(pB_cam);
pB_cam_norm = norm_matB*pB_cam;

% Data sizes.
n_1 = size(pA,2);

% Calculate Q (there is no beta, so only alpha, remembering lab 1).
Q = [pB_cam_norm(1,:).*pA_cam_norm(1,:); pB_cam_norm(1,:).*pA_cam_norm(2,:); pB_cam_norm(1,:); ...
     pB_cam_norm(2,:).*pA_cam_norm(1,:); pB_cam_norm(2,:).*pA_cam_norm(2,:); pB_cam_norm(2,:); ...
     pA_cam_norm(1,:); pA_cam_norm(2,:); ones(1,n_1)]';
 
% Compute eigenvectors (V) and eigenvalues (S) of Q.
[U,S,V] = svd(Q);

% Select the eigenvector h0 with minimun eigenvalue -> H
E_norm = reshape(V(:,size(Q,2)),3,3)';

% Find E (disabling the normalization).
E_no_properties = norm_matB'*E_norm*norm_matA; 

% Fulfill matrix E properties.
[U,S,V] = svd(E_no_properties);
E = U*[0.5*(S(1,1)+S(2,2)) 0 0; 0 0.5*(S(1,1)+S(2,2)) 0; 0 0 0]*V';    



