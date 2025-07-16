%% My Stereo Rectification Implementation
% Alternative implementation of stereo camera rectification
% This code achieves the same results as the original code of the prof but with different
% algorithmic approaches

clear; clc; close all;

stereo_calib = load('calib/Calib_Results_stereo.mat');
left_calib = load('calib/Calib_Results_left.mat');
right_calib = load('calib/Calib_Results_right.mat');

left_img = imread('Scacchiera/left01.jpg');
right_img = imread('Scacchiera/right01.jpg');

world_points = stereo_calib.X_left_1;
num_points = size(world_points, 2);

% Camera intrinsic matrices
K_left = stereo_calib.KK_left;
K_right = stereo_calib.KK_right;

R_stereo = stereo_calib.R; % Rotation left to right
T_stereo = stereo_calib.T; % Translation

% Extrinsic parameters
R_left_ext = my_rodrigues_conversion(stereo_calib.omc_left_1);
T_left_ext = stereo_calib.Tc_left_1;

% Step 1: Compute camera centers using alternative method

P_left_full = compute_projection_matrix(K_left, eye(3), zeros(3, 1));
P_right_full = compute_projection_matrix(K_right, R_stereo, T_stereo);

extrinsic_transform = [R_left_ext, T_left_ext; 0 0 0 1];
world_points_cam = extrinsic_transform * [world_points; ones(1, num_points)];

left_projections = P_left_full * world_points_cam;
right_projections = P_right_full * world_points_cam;

% Convert to image coordinates
left_pixels = dehomogenize_points(left_projections);
right_pixels = dehomogenize_points(right_projections);

%  matrices for rectification
Q_left = P_left_full(:, 1:3);
Q_right = P_right_full(:, 1:3);
q_left = P_left_full(:, 4);
q_right = P_right_full(:, 4);

% optical centers
center_left = compute_camera_center(Q_left, q_left);
center_right = compute_camera_center(Q_right, q_right);

% Step 2: Build rectification rotation matrix using Gram-Schmidt process

baseline_vector = center_right - center_left;
baseline_length = norm(baseline_vector);

original_z_axis = R_left_ext(3, :)'; % Original camera z-axis

% Build orthonormal basis using Gram-Schmidt
new_x = baseline_vector / baseline_length;
temp_y = cross(original_z_axis, new_x);
new_y = temp_y / norm(temp_y);
new_z = cross(new_x, new_y);

R_rect = [new_x'; new_y'; new_z'];

% Step 3: Create rectified projection matrices

K_rect = K_left;

% Build new projection matrices
P_left_rect = K_rect * [R_rect, -R_rect * center_left];
P_right_rect = K_rect * [R_rect, -R_rect * center_right];

% Step 4: Compute homography transformations

H_left = compute_homography(P_left_rect, Q_left);
H_right = compute_homography(P_right_rect, Q_right);

% Step 5: Apply rectification to images

left_rectified = apply_image_transform(left_img, H_left);
right_rectified = apply_image_transform(right_img, H_right);

% Step 6: Transform calibration points

left_rect_points = apply_point_transform(left_projections, H_left);
right_rect_points = apply_point_transform(right_projections, H_right);

% Step 7: Verify rectification quality

% Compute epipolar geometry for rectified cameras
epipole_right = compute_epipole(P_right_rect, P_left_rect);
fundamental_matrix = compute_fundamental_matrix(P_left_rect, P_right_rect, epipole_right);

% Visualization

create_rectification_visualization(left_img, right_img, left_rectified, right_rectified, ...
    left_pixels, right_pixels, left_rect_points, right_rect_points);

% Show epipolar lines
demonstrate_epipolar_geometry(left_rectified, right_rectified, left_rect_points, ...
    right_rect_points, fundamental_matrix);

%% Helper Functions

function P = compute_projection_matrix(K, R, T)
    % Compute projection matrix P = K[R|T]
    P = K * [R, T];
end

function points_2d = dehomogenize_points(points_3d)
    % Convert homogeneous coordinates to Cartesian
    points_2d = [points_3d(1, :) ./ points_3d(3, :);
                 points_3d(2, :) ./ points_3d(3, :)];
end

function center = compute_camera_center(Q, q)
    % Compute camera center using: C = -Q^(-1) * q
    center = -inv(Q) * q;
end

function R = my_rodrigues_conversion(rotation_vector)
    % Alternative Rodrigues formula implementation
    theta = norm(rotation_vector);

    if theta < 1e-8
        R = eye(3);
        return;
    end

    k = rotation_vector / theta; % Unit axis
    K = [0, -k(3), k(2); k(3), 0, -k(1); -k(2), k(1), 0]; % Skew-symmetric matrix

    % Rodrigues formula: R = I + sin(theta)K + (1-cos(theta))K²
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
end

function H = compute_homography(P_new, Q_old)
    % Compute homography H = P_new * Q_old^(-1)
    H = P_new(:, 1:3) * inv(Q_old);
end

function img_transformed = apply_image_transform(img, H)
    % Apply homographic transformation to image
    % Using custom implementation instead of imwarp

    [height, width, channels] = size(img);

    % Create coordinate grids
    [X, Y] = meshgrid(1:width, 1:height);
    coords_homog = [X(:)'; Y(:)'; ones(1, numel(X))];

    % Apply inverse transformation
    H_inv = inv(H);
    transformed_coords = H_inv * coords_homog;

    % Dehomogenize
    X_new = reshape(transformed_coords(1, :) ./ transformed_coords(3, :), height, width);
    Y_new = reshape(transformed_coords(2, :) ./ transformed_coords(3, :), height, width);

    % Interpolate
    img_transformed = zeros(size(img), 'uint8');

    for c = 1:channels
        img_transformed(:, :, c) = uint8(interp2(double(img(:, :, c)), X_new, Y_new, 'linear', 0));
    end

end

function points_transformed = apply_point_transform(points_homog, H)
    % Apply homography to points
    transformed = H * points_homog;
    points_transformed = [transformed(1, :) ./ transformed(3, :);
                          transformed(2, :) ./ transformed(3, :)];
end

function epipole = compute_epipole(P_right, P_left)
    % Compute epipole in right image
    left_center = compute_camera_center(P_left(:, 1:3), P_left(:, 4));
    epipole_homog = P_right * [left_center; 1];
    epipole = epipole_homog(1:3);
end

function F = compute_fundamental_matrix(P_left, P_right, epipole)
    % Compute fundamental matrix
    e_cross = [0, -epipole(3), epipole(2);
               epipole(3), 0, -epipole(1);
               -epipole(2), epipole(1), 0];
    F = e_cross * P_right(:, 1:3) * inv(P_left(:, 1:3));
end

function create_rectification_visualization(img1_orig, img2_orig, img1_rect, img2_rect, ...
        points1_orig, points2_orig, points1_rect, points2_rect)

    figure('Name', 'Rectification Results Comparison', 'Position', [100, 100, 1200, 800]);

    % Original images with points
    subplot(2, 2, 1);
    imshow(img1_orig);
    hold on;
    scatter(points1_orig(1, :), points1_orig(2, :), 30, 'b', 'filled');
    title('Original Left Image with Calibration Points');

    subplot(2, 2, 2);
    imshow(img2_orig);
    hold on;
    scatter(points2_orig(1, :), points2_orig(2, :), 30, 'r', 'filled');
    title('Original Right Image with Calibration Points');

    % Rectified images with points
    subplot(2, 2, 3);
    imshow(img1_rect);
    hold on;
    scatter(points1_rect(1, :), points1_rect(2, :), 30, 'b', 'filled');
    title('Rectified Left Image');

    subplot(2, 2, 4);
    imshow(img2_rect);
    hold on;
    scatter(points2_rect(1, :), points2_rect(2, :), 30, 'r', 'filled');
    title('Rectified Right Image');
end

function demonstrate_epipolar_geometry(img1_rect, img2_rect, points1_rect, points2_rect, F)

    figure('Name', 'Epipolar Lines Verification', 'Position', [200, 200, 1000, 400]);

    % Select a few points for demonstration
    selected_indices = [1, 10, 20, min(30, size(points1_rect, 2))];

    subplot(1, 2, 1);
    imshow(img1_rect);
    hold on;
    scatter(points1_rect(1, selected_indices), points1_rect(2, selected_indices), 60, 'r', 'filled');
    title('Selected Points in Left Rectified Image');

    subplot(1, 2, 2);
    imshow(img2_rect);
    hold on;

    x_range = 1:size(img2_rect, 2);

    for i = selected_indices
        % Compute epipolar line
        point_homog = [points1_rect(:, i); 1];
        epipolar_line = F * point_homog;

        % Plot corresponding point
        scatter(points2_rect(1, i), points2_rect(2, i), 60, 'r', 'filled');

        % Plot epipolar line (should be horizontal after rectification)
        if abs(epipolar_line(2)) > 1e-6 % Avoid division by zero
            y_line =- (epipolar_line(1) * x_range + epipolar_line(3)) / epipolar_line(2);
            plot(x_range, y_line, 'g-', 'LineWidth', 2);
        end

    end

    title('Epipolar Lines in Right Rectified Image (Should be Horizontal)');

    % Verify horizontality
    fprintf('Epipolar line analysis:\n');

    for i = selected_indices
        point_homog = [points1_rect(:, i); 1];
        epipolar_line = F * point_homog;
        slope = -epipolar_line(1) / epipolar_line(2);
        fprintf('Point %d: Epipolar line slope = %.6f (should be almost zero)\n', i, slope);
    end

end
