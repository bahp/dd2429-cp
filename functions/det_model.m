% function model = det_model(cam, data)
%
% Method: Determines the 3D model points by triangulation
%         of a stereo camera system. We assume that the data 
%         is already normalized 
% 
% Input: data (6,n) matrix
%        cam (6,4) matrix
% 
% Output: model (4,n matrix of all n points)
%

function model = det_model(cam, data)

%-------------------------------------------------------------------------
%                         FILL IN THIS PART
%-------------------------------------------------------------------------
% Get the points of each camera.
pA = data(1:3,:);
pB = data(4:6,:);

n = size(data,2);

% General projection matrix M of each camera.
Ma = cam(1:3,:);
Mb = cam(4:6,:);

% Calculate Q(4x4).
ec1 = [(pA(1,:).*Ma(3,1) - Ma(1,1)); (pA(1,:).*Ma(3,2) - Ma(1,2)); ...
       (pA(1,:).*Ma(3,3) - Ma(1,3)); (pA(1,:).*Ma(3,4) - Ma(1,4))]';
ec2 = [(pA(2,:).*Ma(3,1) - Ma(2,1)); (pA(2,:).*Ma(3,2) - Ma(2,2)); ...
       (pA(2,:).*Ma(3,3) - Ma(2,3)); (pA(2,:).*Ma(3,4) - Ma(2,4))]';
  
ec3 = [(pB(1,:).*Mb(3,1) - Mb(1,1)); (pB(1,:).*Mb(3,2) - Mb(1,2)); ...
       (pB(1,:).*Mb(3,3) - Mb(1,3)); (pB(1,:).*Mb(3,4) - Mb(1,4))]';
ec4 = [(pB(2,:).*Mb(3,1) - Mb(2,1)); (pB(2,:).*Mb(3,2) - Mb(2,2)); ...
       (pB(2,:).*Mb(3,3) - Mb(2,3)); (pB(2,:).*Mb(3,4) - Mb(2,4))]';
  
Q = [ec1 ec2 ec3 ec4];

% Get models.
model = zeros(4,n);
for i = 1:n
    % Get W.
    W = reshape(Q(i,:),4,4)';
    
    % Compute eigenvectors (V) and eigenvalues (S) of W.
    [U,S,V] = svd(W);
    
    % Add to models.
    model(:,i) = V(:,end);
end
