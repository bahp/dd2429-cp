% H = det_homographies(points1, points2)
%
% Method: Determines the mapping H * points1 = points2
% 
% Input:  points1, points2 are of the form (3,n) with n is the amount of
%         points. The points should be normalized for better performance 
% 
% Output: H (3,3) matrix 
%

function H = det_homographies(points1, points2)

%-------------------------------------------------------------------------
%                         FILL IN THIS PART
%-------------------------------------------------------------------------
% Theory:  
%         | xa |   | h11 h12 h13 | | xb |
%         | ya | ~ | h21 h22 h23 | | yb |
%         | 1  |   | h31 h32 h33 | | 1  |
% where xb = points1(x), yb = points1(y)
%       xa = points2(x), ya = points2(y)

n_1 = size(points1,2);
n_2 = size(points2,2);

% Compute (9-dimensional): 
% alpha(i) = (xb(i),yb(i),1 ,0 ,0 ,0 ,-xb(i)*xa(i), -yb(i)*xa(i),-xa(i))' 
% beta(i)  = (0, 0, 0,xb(i),yb(i),1,-xb(i)*ya(i), -yb(i)*ya(i) ,-ya(i))'
% Note xa=ref.
alpha = [points1(1,:); points1(2,:); ones(1,n_1); zeros(3,n_1); ...
         -points1(1,:).*points2(1,:); -points1(2,:).*points2(1,:); ...
         -points2(1,:)]';
     
beta = [zeros(3,n_2); points1(1,:); points1(2,:); ones(1,n_2); ...
        -points1(1,:).*points2(2,:); -points1(2,:).*points2(2,:); ...
        -points2(2,:)]';

% With n = number of points build Q [2nx9) = [alpha ; beta]
Q = [alpha; beta];

% Compute eigenvectors and eigenvalues of Q'Q
% S -> diagonal matrix : eigenvalues sorted in decreasing order.
% V -> represents eigenvectors in columns.
[U,S,V] = svd(Q);

% Select the eigenvector h0 with minimun eigenvalue -> H
% WHY I NEED TO PUT -?.
H = -reshape(V(:,size(Q,2)),3,3)';
%-------------------------------------------------------------------------
