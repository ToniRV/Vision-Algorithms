% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C2_W = T_C2_C1 is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 1)
%   to camera 2.
%

function [R,T] = disambiguateRelativePose(Rots,u3,p1,p2,K1,K2)
    T1 = [eye(3), zeros(3,1)];
    M1 = K1*T1;
    % Possible Transformation matrices given by the decomposition of E
    T2s = cat(3, [Rots(:,:,1), u3], [Rots(:,:,1), -u3], [Rots(:,:,2), u3], [Rots(:,:,2), -u3]);
    
    % X represents the 3D points triangulated with each T possible
    X_A = zeros (4, size(p1,2), 4);
    X_B = zeros (4, size(p1,2), 4);
    
    % Counts the number of 3D points which have positive depth components
    positive_counts = zeros (1, 4);
    for i=1:4
        M2 = K2*T2s(:,:,i);
        % Compute points in world coordinates
        X_A(:,:,i) = linearTriangulation(p1, p2, M1, M2);
        % Compute points in camera B coordinates
        X_B(1:3,:,i) = T2s(:,:,i)*X_A(:,:,i);
        % Sum positive points
        positive_counts(i) = sum([X_A(3,:,i)>0, X_B(3,:,i)>0]);
    end
    [~, the_one] = max(positive_counts);
    R = T2s(:, 1:3, the_one);
    T = T2s(:, 4, the_one);
end

