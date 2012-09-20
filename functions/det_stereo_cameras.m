% function [cams, cam_centers] = det_stereo_cameras(E, K1, K2, data); 
%
% Method: Calculate the first and second camera matrix. 
%         The second camera matrix is unique up to scale. 
%         The essential matrix and 
%         the internal camera matrices are known. Furthermore one 
%         point is needed in order solve the ambiguity in the 
%         second camera matrix.
% 
% Input:  E - essential matrix with the singular values (a,a,0) 
%         K1 - internal matrix of the first camera; (3,3) matrix
%         K2 - internal matrix of the second camera; (3,3) matrix
%         data - (6,1) matrix, of 2 points in camera one and two 
% 
% Output: cams (6,4), where cams(1:3,:) is the first and 
%                     cams(4:6,:) is the second camera      
%         cam_centers(4,2) where (:,1) is the first and (:,2) 
%                          the second camera center
%

function [cams, cam_centers] = det_stereo_cameras(E, Ka, Kb, data)

%-------------------------------------------------------------------------
%                         FILL IN THIS PART
%-------------------------------------------------------------------------
% Calculate camera a matrix Ma = Ka(I|0).
Ma = Ka*[eye(3) zeros(3,1)];

% Calculate camera b matrix Mb = KbR(I|t).
% Calculate translation t.
[U,S,V] = svd(E);            % Calculate t.
t = V(:,end)./sum(V(:,end)); % Normalize t.

% Calculate rotation R (R1=IWV' or R2=UW'V') with W=[0 -1 0; 1 0 0; 0 0 1]
W = [0 -1 0; 1 0 0; 0 0 1];
R1 = U*W*V';
R2 = U*W'*V';

% Check the determinant of R1 and R2 is 1.
if (det(R1) == -1) R1 = R1.*-1; end
if (det(R2) == -1) R2 = R2.*-1; end

% Find the 4 possible Mb's.
Mb1 = Kb*R1*[eye(3) t];
Mb2 = Kb*R1*[eye(3) -t];
Mb3 = Kb*R2*[eye(3) t];
Mb4 = Kb*R2*[eye(3) -t];

% Check which pair [Ma; Mb] has the point in front of both cameras.
p1 = det_model([Ma; Mb1],data(:,1))';
p2 = det_model([Ma; Mb2],data(:,1))';
p3 = det_model([Ma; Mb3],data(:,1))';
p4 = det_model([Ma; Mb4],data(:,1))';

% Normalize the points (needed for check if is in front a camera or not).
p1 = p1./p1(4);
p2 = p2./p2(4);
p3 = p3./p3(4);
p4 = p4./p4(4);

% Find the aligned points (with or without -?).
aligned_point1 = R1*[eye(3) t]*p1';
aligned_point2 = R1*[eye(3) -t]*p2';
aligned_point3 = R2*[eye(3) t]*p3';
aligned_point4 = R2*[eye(3) -t]*p4';

% Choose the point that is positive in both camA and camB.
cam_centers = [0 0 0 1; t' 1]';
if ( (sign(p1(3)) == 1) && (sign(aligned_point1(3)) == 1) )   
    disp('    OPTION 1 - In front of camera');
    cams = [Ma; Mb1];
end

if ( (sign(p2(3)) == 1) && (sign(aligned_point2(3)) == 1) )  
    disp('    OPTION 2 - In front of camera');   
    cams = [Ma; Mb2];
end

if ( (sign(p3(3)) == 1) && (sign(aligned_point3(3)) == 1) )  
    disp('    OPTION 3 - In front of camera');
    cams = [Ma; Mb3];
end

if ( (sign(p4(3)) == 1) && (sign(aligned_point4(3)) == 1) )  
    disp('    OPTION 4 - In front of camera');
    cams = [Ma; Mb4];
end

% Verification.
% disp('Check E = R[0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0] (normalized)');
% disp('E matrix');
% disp(E./sum(sum(E)))
% m = R1*[0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0];
% disp('R*[0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0]');
% disp(m./sum(sum(m)));