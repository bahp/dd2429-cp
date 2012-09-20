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

function [cams, cam_centers] = det_stereo_cameras(E, K1, K2, data)

%------------------------------
%
% FILL IN THIS PART
%
%------------------------------
cam_centers = zeros(4,2);
cams = zeros(6,4);

cam_centers(1:3,1) = 0;
cam_centers(4,1) = 1; %CHECK

[U, S, V] = svd(E);

%V = V';

t = V(:,3);

t = t./(t'*t); %Normalized

W = [0 -1 0; 1 0 0; 0 0 1];

R1 = U * W * V';
R2 = U * W' * V';

r1 = det(R1);
r2 = det(R2);

if r1 < 0
    R1 = R1 * (-1);
end
if r2 < 0
    R2 = R2 * (-1);
end

center1_3 = t;
center2_4 = -t;

Mb1 = K2 * R1 * [eye(3) -center1_3];
Mb2 = K2 * R1 * [eye(3) -center2_4];
Mb3 = K2 * R2 * [eye(3) -center1_3];
Mb4 = K2 * R2 * [eye(3) -center2_4];

cams1(1:3, :) = K1 * [eye(3) [0 0 0]'];
cams1(4:6, :) = Mb1;
cams2(1:3, :) = K1 * [eye(3) [0 0 0]'];
cams2(4:6, :) = Mb2;
cams3(1:3, :) = K1 * [eye(3) [0 0 0]'];
cams3(4:6, :) = Mb3;
cams4(1:3, :) = K1 * [eye(3) [0 0 0]'];
cams4(4:6, :) = Mb4;

P1 = det_model(cams1,data(:, 1));
P2 = det_model(cams2,data(:, 1));
P3 = det_model(cams3,data(:, 1));
P4 = det_model(cams4,data(:, 1));


if (P1(3) > 0) && ...
        (dot((P1(1:3) - center1_3), R1(3,:)) > 0)
    cam_centers(1:3,2) = center1_3;
    cam_centers(4,2) = 1;
    cams = cams1;
end
if (P2(3) > 0) && ...
        (dot((P2(1:3) - center2_4), R1(3,:)) > 0)
    
    cams = cams2;
    cam_centers(1:3,2) = center2_4;
    cam_centers(4,2) = 1;
end
if (P3(3) > 0) && ...
        (dot((P3(1:3) - center1_3), R2(3,:)) > 0)
    
    cams = cams3;
    cam_centers(1:3,2) = center1_3;
    cam_centers(4,2) = 1;
end
if (P4(3) > 0) && ...
        (dot((P4(1:3) - center2_4), R2(3,:)) > 0)
    cams = cams4;
    cam_centers(1:3,2) = center2_4;
    cam_centers(4,2) = 1;
end