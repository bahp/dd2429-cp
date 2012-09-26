% H = det_rectification_matrix(points1, points2)
%
% Method: Determines the mapping H * points1 = points2
% 
% Input: points1, points2 of the form (4,n) 
%        n has to be at least 5
%
% Output:  H (4,4) matrix 
% 

function H = det_rectification_matrix(p, p_prim)

%------------------------------------------------------------------------
%                          FILL IN THIS PART
%------------------------------------------------------------------------
% Useful variables.
n_points = size(p,2);  % Total number of points.
k = n_points;                 % Points needed to get 15 eq -> solve H.

% Find equations alpha, beta and gamma matrix.
% h = [h11 h12 h12 h14 
%      h21 h22 h23 h24 
%      h31 h32 h33 h33 
%      h41 h42 
%      h43 h44]
alpha = [p(1,1:k); p(2,1:k); p(3,1:k); p(4,1:k); 
         zeros(4,k);
         zeros(4,k);
         -p(1,1:k).*p_prim(1,1:k); -p(2,1:k).*p_prim(1,1:k); ... 
         -p(3,1:k).*p_prim(1,1:k); -p(4,1:k).*p_prim(1,1:k)]';
     
beta  = [zeros(4,k);
         p(1,1:k); p(2,1:k); p(3,1:k); p(4,1:k); ...
         zeros(4,k);
         -p(1,1:k).*p_prim(2,1:k); -p(2,1:k).*p_prim(2,1:k); ... 
         -p(3,1:k).*p_prim(2,1:k); -p(4,1:k).*p_prim(2,1:k)]';
     
gamma = [zeros(4,k);
         zeros(4,k);
         p(1,1:k); p(2,1:k); p(3,1:k); p(4,1:k); ...
         -p(1,1:k).*p_prim(3,1:k); -p(2,1:k).*p_prim(3,1:k); ... 
         -p(3,1:k).*p_prim(3,1:k); -p(4,1:k).*p_prim(3,1:k)]';
     
% Create W matrix.
W = [alpha; beta; gamma];

% Compute eigenvectors and eigenvalues of W
[U,S,V] = svd(W);

% Select the eigenvector h0 with minimun eigenvalue -> H
H = reshape(V(:,end),4,4)';