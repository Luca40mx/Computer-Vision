clc; clearvars; close all;

image1 = imread('datasets/triple_images/aarhus1.jpg');
image2 = imread('datasets/triple_images/aarhus2.jpg');
image3 = imread('datasets/triple_images/aarhus3.jpg');

% Detect features in all three images
gray1 = im2gray(image1);
points1 = detectSIFTFeatures(gray1);

gray2 = im2gray(image2);
points2 = detectSIFTFeatures(gray2);

gray3 = im2gray(image3);
points3 = detectSIFTFeatures(gray3);

%% Extract feature descriptors
[features1, validPoints1] = extractFeatures(gray1, points1);
[features2, validPoints2] = extractFeatures(gray2, points2);
[features3, validPoints3] = extractFeatures(gray3, points3);

%% Match features between image1 and image2
indices12 = matchFeatures(features1, features2, "Method", "Exhaustive");

matchPoints1 = validPoints1(indices12(:, 1));
matchPoints2 = validPoints2(indices12(:, 2));

figure(1);
showMatchedFeatures(image1, image2, matchPoints1, matchPoints2, "montage");
title('Matches between Image 1 and Image 2');

%% Match features between image2 and image3
indices23 = matchFeatures(features2, features3, "Method", "Exhaustive");

matchPoints2_for3 = validPoints2(indices23(:, 1));
matchPoints3 = validPoints3(indices23(:, 2));

figure(2);
showMatchedFeatures(image2, image3, matchPoints2_for3, matchPoints3, "montage");
title('Matches between Image 2 and Image 3');

%% RANSAC for image1-image2
[tform12, inlierIdx12] = estgeotform2d(matchPoints1, matchPoints2, "projective", "MaxNumTrials", 10000, "Confidence", 99.9);
inliers1 = matchPoints1(inlierIdx12, :);
inliers2 = matchPoints2(inlierIdx12, :);
figure(3);
showMatchedFeatures(image1, image2, inliers1, inliers2, "montage");
title('Inliers between Image 1 and Image 2');

%% RANSAC for image2-image3
[tform23, inlierIdx23] = estgeotform2d(matchPoints2_for3, matchPoints3, "projective");
inliers2_for3 = matchPoints2_for3(inlierIdx23, :);
inliers3 = matchPoints3(inlierIdx23, :);
figure(4);
showMatchedFeatures(image2, image3, inliers2_for3, inliers3, "montage");
title('Inliers between Image 2 and Image 3');

%%%%%%%%%%%%%%%%%%%%%%
% Extract homography matrices from MATLAB's built-in function
H12_matlab = tform12.A;
H23_matlab = tform23.A;

% Compute homography with custom function for image1-image2
H12_custom = double(compute_homography_matrix(inliers1.Location(:, 1), inliers1.Location(:, 2), ...
    inliers2.Location(:, 1), inliers2.Location(:, 2)));

% Compute homography with custom function for image2-image3
H23_custom = double(compute_homography_matrix(inliers2_for3.Location(:, 1), inliers2_for3.Location(:, 2), ...
    inliers3.Location(:, 1), inliers3.Location(:, 2)));

% Normalize matrices for comparison
H12_matlab_norm = H12_matlab / H12_matlab(3, 3);
H12_custom_norm = H12_custom / H12_custom(3, 3);
H23_matlab_norm = H23_matlab / H23_matlab(3, 3);
H23_custom_norm = H23_custom / H23_custom(3, 3);

% Calculate the differences
diff_matrix12 = abs(H12_matlab_norm - H12_custom_norm);
diff_matrix23 = abs(H23_matlab_norm - H23_custom_norm);
fprintf('Absolute difference between normalized matrices (Image 1-2):\n');
disp(diff_matrix12);
fprintf('Absolute difference between normalized matrices (Image 2-3):\n');
disp(diff_matrix23);
%%%%%%%%%%%%%%%%%%%%%%

%% Color blending for 3 images

% Strategy: Use image2 as the reference (center image)
% Transform image1 using tform12 (maps image1 to image2 space)
% Transform image3 using inverse of tform23 (maps image3 to image2 space)

% Invert tform23 to map from image3 to image2
tform32 = invert(tform23);

% STEP 1: Find the output bounds that contain all three images

% Get bounds for image1 warped to image2 space
[xlimits1, ylimits1] = outputLimits(tform12, [1 size(image1, 2)], [1 size(image1, 1)]);

% Get bounds for image3 warped to image2 space
[xlimits3, ylimits3] = outputLimits(tform32, [1 size(image3, 2)], [1 size(image3, 1)]);

% Combine all bounds including image2 (reference)
xlimits = [min([xlimits1(1), 1, xlimits3(1)]), max([xlimits1(2), size(image2, 2), xlimits3(2)])];
ylimits = [min([ylimits1(1), 1, ylimits3(1)]), max([ylimits1(2), size(image2, 1), ylimits3(2)])];

% Calculate output image size
width = ceil(xlimits(2) - xlimits(1));
height = ceil(ylimits(2) - ylimits(1));

% Create output spatial reference
outputRef = imref2d([height, width], xlimits, ylimits);

% Warp image1 to the output space
warped_image1 = imwarp(image1, tform12, 'OutputView', outputRef);
mask1 = imwarp(true(size(image1, 1), size(image1, 2)), tform12, 'OutputView', outputRef);

% Warp image3 to the output space
warped_image3 = imwarp(image3, tform32, 'OutputView', outputRef);
mask3 = imwarp(true(size(image3, 1), size(image3, 2)), tform32, 'OutputView', outputRef);

% Place image2 in the output space (no transformation needed)
warped_image2 = zeros(height, width, size(image2, 3), 'like', image2);
mask2 = false(height, width);

% Calculate where image2 should be placed in the output coordinate system
x_start = max(1, 1 - round(xlimits(1)) + 1);
y_start = max(1, 1 - round(ylimits(1)) + 1);
x_end = min(width, x_start + size(image2, 2) - 1);
y_end = min(height, y_start + size(image2, 1) - 1);

% Calculate corresponding indices in image2
img2_x_start = max(1, round(xlimits(1)) - 1 + 1);
img2_y_start = max(1, round(ylimits(1)) - 1 + 1);
img2_x_end = min(size(image2, 2), img2_x_start + (x_end - x_start));
img2_y_end = min(size(image2, 1), img2_y_start + (y_end - y_start));

warped_image2(y_start:y_end, x_start:x_end, :) = image2(img2_y_start:img2_y_end, img2_x_start:img2_x_end, :);
mask2(y_start:y_end, x_start:x_end) = true;

% Convert images to double for blending calculations
warped_image1 = im2double(warped_image1);
warped_image2 = im2double(warped_image2);
warped_image3 = im2double(warped_image3);

% STEP 2: Create weight maps for blending
weight1 = double(mask1);
weight2 = double(mask2);
weight3 = double(mask3);

% Apply distance-based weighting for smooth transitions
% Create distance transforms
dist1 = bwdist(~mask1);
dist2 = bwdist(~mask2);
dist3 = bwdist(~mask3);

% Find overlap regions
overlap12 = mask1 & mask2 & ~mask3;
overlap23 = mask2 & mask3 & ~mask1;
overlap123 = mask1 & mask2 & mask3;

% Handle image1-image2 overlap
if any(overlap12(:))
    total_dist = dist1 + dist2;
    total_dist(total_dist == 0) = 1;
    weight1(overlap12) = dist1(overlap12) ./ total_dist(overlap12);
    weight2(overlap12) = dist2(overlap12) ./ total_dist(overlap12);
    weight3(overlap12) = 0;
end

% Handle image2-image3 overlap
if any(overlap23(:))
    total_dist = dist2 + dist3;
    total_dist(total_dist == 0) = 1;
    weight2(overlap23) = dist2(overlap23) ./ total_dist(overlap23);
    weight3(overlap23) = dist3(overlap23) ./ total_dist(overlap23);
    weight1(overlap23) = 0;
end

% Handle triple overlap (all three images)
if any(overlap123(:))
    total_dist = dist1 + dist2 + dist3;
    total_dist(total_dist == 0) = 1;
    weight1(overlap123) = dist1(overlap123) ./ total_dist(overlap123);
    weight2(overlap123) = dist2(overlap123) ./ total_dist(overlap123);
    weight3(overlap123) = dist3(overlap123) ./ total_dist(overlap123);
end

% Perform weighted blending
blended_image = zeros(size(warped_image1), 'like', warped_image1);
total_weight = weight1 + weight2 + weight3;

% Avoid division by zero
valid_pixels = total_weight > 0;

for c = 1:size(warped_image1, 3)
    channel1 = warped_image1(:, :, c);
    channel2 = warped_image2(:, :, c);
    channel3 = warped_image3(:, :, c);

    blended_channel = (weight1 .* channel1 + weight2 .* channel2 + weight3 .* channel3);
    blended_channel(valid_pixels) = blended_channel(valid_pixels) ./ total_weight(valid_pixels);

    blended_image(:, :, c) = blended_channel;
end

%% Results

figure(40);
imshow(blended_image);
title('Final Panoramic Image - 3 Images Stitched');

figure(50);
imshow(warped_image1);
title('Warped Image 1');

figure(51);
imshow(warped_image2);
title('Image 2 (Reference)');

figure(52);
imshow(warped_image3);
title('Warped Image 3');

figure(140);
weight_viz = cat(3, weight1, weight2, weight3);
imshow(weight_viz);
title('Weight Visualization (R=img1, G=img2, B=img3)');

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
