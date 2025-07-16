clc; clearvars; close all;

% image1 = imread('lab1.png');
% image2 = imread('lab2.png');

image1 = imread('image1.png');
image2 = imread('image2.png');

gray1 = im2gray(image1);
points1 = detectSIFTFeatures(gray1);

gray2 = im2gray(image2);
points2 = detectSIFTFeatures(gray2);

%% Extract feature descriptors
[features1, validPoints1] = extractFeatures(gray1, points1);
[features2, validPoints2] = extractFeatures(gray2, points2);

%% match features
indices = matchFeatures(features1, features2, "Method", "Exhaustive");

matchPoints1 = validPoints1(indices(:, 1));
matchPoints2 = validPoints2(indices(:, 2));

showMatchedFeatures(image1, image2, matchPoints1, matchPoints2, "montage");

%% RANSAC
[tform, inlierIdx] = estgeotform2d(matchPoints1, matchPoints2, "projective");
inliers1 = matchPoints1(inlierIdx, :);
inliers2 = matchPoints2(inlierIdx, :);
figure(4);
showMatchedFeatures(image1, image2, inliers1, inliers2, "montage");

%%%%%%%%%%%%%%%%%%%%%%
% Extract homography matrix from MATLAB's built-in function
H_matlab = tform.A;

% Compute homography with my function
H_custom = double(compute_homography_matrix(inliers1.Location(:, 1), inliers1.Location(:, 2), ...
    inliers2.Location(:, 1), inliers2.Location(:, 2)));

% Normalize both matrices for comparison (homography is defined up to scale)
H_matlab_norm = H_matlab / H_matlab(3, 3);
H_custom_norm = H_custom / H_custom(3, 3);

% Calculate the difference between normalized matrices
diff_matrix = abs(H_matlab_norm - H_custom_norm);
fprintf('Absolute difference between normalized matrices:\n');
disp(diff_matrix);
%%%%%%%%%%%%%%%%%%%%%%

%% Homography
image_fixed = imref2d(size(image2));
[a, b] = imwarp(image1, tform);
figure(100);
imshowpair(image2, image_fixed, a, b, "blend", "Scaling", "joint");

%% Color blending

% STEP 1: We need to find the output bounds that contain both images and

[xlimits, ylimits] = outputLimits(tform, [1 size(image1, 2)], [1 size(image1, 1)]);

% Expand limits to include image2
xlimits(1) = min(xlimits(1), 1);
ylimits(1) = min(ylimits(1), 1);
xlimits(2) = max(xlimits(2), size(image2, 2));
ylimits(2) = max(ylimits(2), size(image2, 1));

% Calculate output image size
width = ceil(xlimits(2) - xlimits(1));
height = ceil(ylimits(2) - ylimits(1));

% Create output spatial reference
outputRef = imref2d([height, width], xlimits, ylimits);

% Warp image1 to the output space
warped_image1 = imwarp(image1, tform, 'OutputView', outputRef);

% Create a mask for the warped image1 (to know which pixels are valid)
mask1 = imwarp(true(size(image1, 1), size(image1, 2)), tform, 'OutputView', outputRef);

% Place image2 in the output space
warped_image2 = zeros(height, width, size(image2, 3), 'like', image2);
mask2 = false(height, width);

% Calculate where image2 should be placed in the output coordinate system
x_start = max(1, 1 - round(xlimits(1)) + 1);
y_start = max(1, 1 - round(ylimits(1)) + 1);
x_end = min(width, x_start + size(image2, 2) - 1);
y_end = min(height, y_start + size(image2, 1) - 1);

% Place image2 in the output space
img2_x_start = max(1, round(xlimits(1)) - 1 + 1);
img2_y_start = max(1, round(ylimits(1)) - 1 + 1);
img2_x_end = min(size(image2, 2), img2_x_start + (x_end - x_start));
img2_y_end = min(size(image2, 1), img2_y_start + (y_end - y_start));

warped_image2(y_start:y_end, x_start:x_end, :) = image2(img2_y_start:img2_y_end, img2_x_start:img2_x_end, :);
mask2(y_start:y_end, x_start:x_end) = true;

% Convert images to double for blending calculations
warped_image1 = im2double(warped_image1);
warped_image2 = im2double(warped_image2);

%   STEP 2: Create weight maps for blending
% Weight decreases from center towards edges to create smooth transitions
weight1 = double(mask1);
weight2 = double(mask2);

% Apply distance-based weighting in overlap regions
overlap_mask = mask1 & mask2;

if any(overlap_mask(:))
    % Create distance transforms for smooth blending
    dist1 = bwdist(~mask1);
    dist2 = bwdist(~mask2);

    % Normalize distances in overlap region
    total_dist = dist1 + dist2;
    total_dist(total_dist == 0) = 1; % Avoid division by zero

    % Update weights in overlap region for smooth blending
    weight1(overlap_mask) = dist1(overlap_mask) ./ total_dist(overlap_mask);
    weight2(overlap_mask) = dist2(overlap_mask) ./ total_dist(overlap_mask);
end

% Perform weighted blending: I_blend = (w1*I1 + w2*I2)/(w1 + w2)
blended_image = zeros(size(warped_image1), 'like', warped_image1);
total_weight = weight1 + weight2;

% Avoid division by zero
valid_pixels = total_weight > 0;

for c = 1:size(warped_image1, 3)
    channel1 = warped_image1(:, :, c);
    channel2 = warped_image2(:, :, c);

    blended_channel = (weight1 .* channel1 + weight2 .* channel2);
    blended_channel(valid_pixels) = blended_channel(valid_pixels) ./ total_weight(valid_pixels);

    blended_image(:, :, c) = blended_channel;
end

%% Results

figure(40);
imshow(blended_image);
title('Final Panoramic Image');

figure(140);
weight_viz = cat(3, weight1, weight2, zeros(size(weight1)));
imshow(weight_viz);
title('Weight Visualization');

function H_matrix = compute_homography_matrix(xs, ys, xd, yd)
    % Compute homography matrix using SVD
    % xs, ys: source image points
    % xd, yd: destination image points
    % Minimum 4 point correspondences required

    % Ensure we have at least 4 points
    num_points = min(length(xs), length(ys));
    num_points = min(num_points, min(length(xd), length(yd)));

    if num_points < 4
        error('At least 4 point correspondences are required');
    end

    ys = ys(1:num_points);
    xd = xd(1:num_points);
    yd = yd(1:num_points);

    % Build the constraint matrix A for the equation Ah = 0
    % Each point correspondence gives us 2 equations
    A = zeros(2 * num_points, 9);

    for i = 1:num_points
        % First equation: xd_i = (h11*xs_i + h12*ys_i + h13) / (h31*xs_i + h32*ys_i + h33)
        % Rearranged: h11*xs_i + h12*ys_i + h13 - xd_i*h31*xs_i - xd_i*h32*ys_i - xd_i*h33 = 0
        A(2 * i - 1, :) = [xs(i), ys(i), 1, 0, 0, 0, -xd(i) * xs(i), -xd(i) * ys(i), -xd(i)];

        % Second equation: yd_i = (h21*xs_i + h22*ys_i + h23) / (h31*xs_i + h32*ys_i + h33)
        % Rearranged: h21*xs_i + h22*ys_i + h23 - yd_i*h31*xs_i - yd_i*h32*ys_i - yd_i*h33 = 0
        A(2 * i, :) = [0, 0, 0, xs(i), ys(i), 1, -yd(i) * xs(i), -yd(i) * ys(i), -yd(i)];
    end

    % Solve Ah = 0 using SVD
    % The solution is the last column of V (corresponding to smallest singular value)
    [~, ~, V] = svd(A);
    h = V(:, end);

    % Reshape the solution vector into a 3x3 homography matrix
    H_matrix = reshape(h, 3, 3)';

    % Normalize the matrix so that H(3,3) = 1
    H_matrix = H_matrix / H_matrix(3, 3);
end
