% function F = det_F_matrix(points1, points2);
%
% Method: Calculate the F matrix between two views from
%         point correspondences: points2^T * F * points1 = 0
%         we use the normalize 8-point algorithm and 
%         enforce the constraint that the last singular value is 0.
%         The data will be normalized here. 
%         Finally we will check how good the epipolar constraints:
%         points2^T * F * points1 = 0 are fullfilled.
% 
% Input:  points1, points2 of size (3,n) 
%
% Output: F (3,3) with the last singular value 0
% 

function F = det_F_matrix(pA, pB, NORM)

%-------------------------------------------------------------------------
%                        FILL IN THIS PART
%-------------------------------------------------------------------------
%NORM = 1; % Variable to set if normalize or not.

if NORM
    % Points normalized.
    [norm_matA] = get_normalization_matrices(pA);
    pA_norm = norm_matA*pA;

    [norm_matB] = get_normalization_matrices(pB);
    pB_norm = norm_matB*pB;
else
    % Points not normalized.
    pA_norm = pA;
    pB_norm = pB;
end

% Data sizes.
n_1 = size(pA,2);

% Calculate Q (there is no beta, so only alpha, remembering lab 1).
Q = [pB_norm(1,:).*pA_norm(1,:); pB_norm(1,:).*pA_norm(2,:); pB_norm(1,:); ...
     pB_norm(2,:).*pA_norm(1,:); pB_norm(2,:).*pA_norm(2,:); pB_norm(2,:); ...
     pA_norm(1,:); pA_norm(2,:); ones(1,n_1)]';
 
% Compute eigenvectors (V) and eigenvalues (S) of Q.
[U,S,V] = svd(Q);

% Select the eigenvector h0 with minimun eigenvalue -> H
F_norm = reshape(V(:,end),3,3)';

% Undo the normalization (only if we did it).
if NORM  F_no_properties = norm_matB'*F_norm*norm_matA;
else     F_no_properties = F_norm;
end

% Fulfill the property rank = 2.
if (rank(F_no_properties) > 2 )
    k = 2; % rank.
    [U,S,V] = svd(F_no_properties);
    F = U(:, 1:k)* S(1:k, 1:k)* V(:, 1:k)';
else
    F = F_no_properties;
end