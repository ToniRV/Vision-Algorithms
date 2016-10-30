function AR_wireframe_cube ()
    f = imread ('data/images_undistorted/img_0001.jpg');
    f_gray = rgb2gray(f);
    figure('Name', 'Grayscale image'), imshow(f_gray, []);
    hold on
    
    % Create matrix with points of checkerboard in world frame.
    [x_w, y_w] = meshgrid(0:0.04:0.32, 0:0.04:0.20);
    corners = ones(size(x_w, 1), size(x_w, 2), 3);
    corners(:,:,1) = x_w;
    corners(:,:,2) = y_w;
    corners(:,:,3) = zeros(size(x_w, 1), size(x_w, 2));
    
    % Create geometry of a cube
    [cube_faces, cube_cols] = createCube (0.16, 0, -0.08, 0.08);
    
    % Import camera poses
    poses = import_poses('data/poses.txt');
    
    % Import calibration matrix
    K = import_calibration('data/K.txt');
    
    for n = 1%:size(poses, 1)
        [R, T] = poseVector2TransformationMatrix (poses(n,:));
        corners_pixels = world2pixels(K, R, T, corners);
        for i = 1:size(cube_faces,4)
            cube_pixels = world2pixels(K, R, T, cube_faces(:,:,:,i));
            projectLines(cube_pixels);
        end
        for i = 1:size(cube_cols,4)
            cube_pixels = world2pixels(K, R, T, cube_cols(:,:,:,i));
            projectLines(cube_pixels);
        end
        projectPoints(corners_pixels);
    end
end

function pixel_coords = world2pixels (K, R, T, world_points)
    M = K*[R, T];
    pixel_coords = [];
    for i = 1:size(world_points, 1)
        for j = 1:size(world_points, 2)
            scaled_pixel_coords = M*[world_points(i, j, 1); world_points(i, j, 2); world_points(i, j, 3); 1];
            u = scaled_pixel_coords(1)/scaled_pixel_coords(3);
            v = scaled_pixel_coords(2)/scaled_pixel_coords(3);
            pixel_coords = [pixel_coords; [u, v]];
        end
    end
end

function projectPoints(pixel_coords)
    scatter (pixel_coords(:,1), pixel_coords(:,2));
end

function projectLines(pixel_coords)
    line(pixel_coords(:,1)',pixel_coords(:,2)', 'color', 'red', 'linewidth', 3);
end

function [faces, columns] = createCube(x_0, y_0, z_0, size)
    face_1 = horizontal_square(x_0, y_0, z_0, size);
    face_2 = horizontal_square(x_0, y_0, z_0+size, size);
    column_1 = vertical_column(x_0, y_0, z_0, size);
    column_2 = vertical_column(x_0+size, y_0, z_0, size);
    column_3 = vertical_column(x_0, y_0+size, z_0, size);
    column_4 = vertical_column(x_0+size, y_0+size, z_0, size);
    [x, y, z] = meshgrid([x_0, x_0+size], [y_0, y_0+size], [z_0, z_0+size]);
    faces = cat(2, face_1, face_2);
    columns = cat(4, column_1, column_2, column_3, column_4);
end

% Creates a vertical column in 3D starting from given x, y position with a
% given size
function column = vertical_column(x, y, z, size)
    column = ones(1,2,3);
    column(:,:,1) = [x, x];
    column(:,:,2) = [y, y];
    column(:,:,3) = [z, z+size];
end

% Creates horizontal square with given size at height z_0 and origin x_0, y_0
function square = horizontal_square(x_0, y_0, z_0, size)
    square = ones(1, 5, 3);
    
    % Scale
    square(:,:,1) = size*[0, 1, 1, 0, 0]; 
    square(:,:,2) = size*[0, 0, 1, 1, 0];
    
    % Reposition
    square(:,:,1) = square(:,:,1)+x_0;
    square(:,:,2) = square(:,:,2)+y_0;
    
    square(:,:,3) = [z_0 , z_0, z_0, z_0, z_0];
end

function [R, T]= poseVector2TransformationMatrix(x)
    if ~isvector(x)
        error('Input must be a vector')
    else
        [M, N] = size(x);
        if M ~= 6 && N ~= 6
            error ('Input must have six values')
        elseif N == 6
            x = x';
        end
    end
    R = axisAngle2RotationalMatrix (x(1:3));
    T = x(4:6);
end

function R = axisAngle2RotationalMatrix(w)
    if ~isvector(w)
        error('Input must be a vector')
    else
        [M, N] = size(w);
        if M ~= 3 && N ~= 3
            error ('Input must have three values')
        elseif N == 3
            w = w';
        end
    end
    
    theta = norm(w);
    k = w/theta;
    
    % Cross product matrix for k
    k_x = [0        -k(3)   k(2);
           k(3)     0       -k(1);
           -k(2)    k(1)    0];
       
    R = eye(3) + sin(theta)*k_x + (1-cos(theta))*k_x*k_x;
end