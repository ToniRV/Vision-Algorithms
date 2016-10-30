function AR_wireframe_cube ()
    
    % Import camera poses
    poses = import_poses('data/poses.txt');
    
    % Load camera intrinsics
    K = load('data/K.txt'); % calibration matrix      [3x3]
    D = load('data/D.txt'); % distortion coefficients [2x1]

    % Load one image with a given index
    img_index = 1;
    f_gray = rgb2gray(imread(['data/images/',sprintf('img_%04d.jpg',img_index)]));
    
    figure('Name', 'Grayscale image'), imshow(f_gray, []);
    hold on;
    
    % Create matrix with points of checkerboard in world frame.
    [x_w, y_w] = meshgrid(0:0.04:0.32, 0:0.04:0.20);
    total_points = size(x_w, 1)*size(x_w, 2);
    corners = [reshape(x_w, 1, total_points);
               reshape(y_w, 1, total_points);
               zeros(1, total_points)
              ];
    
    % Create geometry of a cube
    x_0 = 0.12;
    y_0 = 0.12;
    z_0 = -0.08;
    size_edge = 0.08;
    edges = createCube (x_0, y_0, z_0, size_edge);
    
    for n = 1%:size(poses, 1)
        [R, T] = poseVector2TransformationMatrix (poses(n,:));
        corners_pixels = world2pixels(K, R, T, corners);
        projectPoints(corners_pixels);
        for k = 1:size(edges,3)
            edge_cube_pixels = world2pixels(K, R, T, edges(:,:,k));
            projectEdge(edge_cube_pixels);
        end
    end
end

% Calculates the pixel coordinates of the world points, given extrinsic and
% intrinsic parameters of the camera.
function pixel_coords = world2pixels (K, R, T, world_points)
    M = K*[R, T];
    scaled_pixel_coords = M*[world_points; ones(1, size(world_points,2))];
    u = scaled_pixel_coords(1,:)./scaled_pixel_coords(3,:);
    v = scaled_pixel_coords(2,:)./scaled_pixel_coords(3,:);
    pixel_coords = [u; v];
end

% Projects points into the graph or image currently hold.
function projectPoints(pixel_coords)
    scatter (pixel_coords(1,:), pixel_coords(2,:), 'filled');
end

% Projects an edge into the graph or image currently hold.
function projectEdge(edge_coords)
    width = 3;
    line(edge_coords(1, :),...
         edge_coords(2, :),...
         'color', 'red', 'linewidth', width);
end

% Outputs a set of edges defining a cube, the edges are given as
% 3-by-2-by-12 matrix containing the x, y and z coordinates of each
% pair of vertex defining an edge, for the 12 edges that there are in a cube.
function edges = createCube(x_0, y_0, z_0, length)
    % Features
    faces = cat(3,...
                horizontal_square(x_0, y_0, z_0, length),...
                horizontal_square(x_0, y_0, z_0+length, length));
    columns = cat(3,...
                vertical_column(x_0, y_0, z_0, length),...
                vertical_column(x_0+length, y_0, z_0, length),...
                vertical_column(x_0, y_0+length, z_0, length),...
                vertical_column(x_0+length, y_0+length, z_0, length));
    
    % Make edges out of points in features.
    edges = cat(3, getEdgesFromFeature(faces(:,:,1)),...
                   getEdgesFromFeature(faces(:,:,2)),...
                   getEdgesFromFeature(columns(:,:,1)),...
                   getEdgesFromFeature(columns(:,:,2)),...
                   getEdgesFromFeature(columns(:,:,3)),...
                   getEdgesFromFeature(columns(:,:,4))...
                );
end

% Creates a vertical column in 3D starting from given x_0, y_0, z_0 position with a
% given size
function column = vertical_column(x_0, y_0, z_0, size)
    column = [[x_0, x_0];
             [y_0, y_0];
             [z_0, z_0+size]];
end

% Creates horizontal square with given size and origin x_0, y_0, z_0.
function square = horizontal_square(x_0, y_0, z_0, size)
    square = ones(3, 4);
    
    % Scale
    square(1, :) = size*[0, 1, 1, 0]; 
    square(2, :) = size*[0, 0, 1, 1];
    
    % Reposition
    square(1, :) = square(1, :)+x_0;
    square(2, :) = square(2, :)+y_0;
    
    square(3, :) = [z_0 , z_0, z_0, z_0];
end

% Extract edges from set of points of closed features, such as a face of a
% square
function edges = getEdgesFromFeature(feature)
    number_of_edges = size(feature, 2);
    number_of_points = 2*number_of_edges;
    
    edges = zeros(3, 2, number_of_edges);
    way_points = zeros(3, number_of_points);
    
    % Feature is closed
    way_points(:, 1) = feature(:, 1);
    way_points(:, number_of_points) = feature(:, 1);
    
    i = 2;
    h = 2;
    while(h <= number_of_edges)
        way_points(:, i) = feature(:, h);
        way_points(:, i+1) = feature(:, h);
        i = i+2;
        h = h+1;
    end
    
    % Store edges as pairs of waypoints
    h = 0;
    for i = 1:2:size(way_points, 2)-1
        h = h + 1;
        edges(:,:,h)=[way_points(:, i), way_points(:, i+1)];
    end
end

% Computes rotational matrix and translational vector that transform world
% points into camera frame points. The input is the pose of the camera
% frame as (w_x, w_y, w_z, t_x, t_y, t_z), where w_# is the axis-angle
% representation of rotation between frames while t_# is the translational
% part.
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

% Computes rotational matrix out of axis-angle representation of
% rotation.
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
