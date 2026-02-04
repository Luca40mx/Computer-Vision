clc; clearvars; close all;
warning('off', 'all');

%% CONFIGURATION
config = struct();
config.IMAGE_FOLDER = 'datasets/test5';
config.NUM_IMAGES = 35;
config.CENTER_IMAGE_INDEX = fix(config.NUM_IMAGES / 2);
config.SCALE_FACTOR = 0.5;
config.OUTPUT_FILENAME = 'panorama_test5.jpg';
config.FEATURE_METHOD = 'SIFT';
config.MATCH_METHOD = 'Approximate';
config.TRANSFORM_TYPE = 'affine';
config.MAX_PANORAMA_DIM = 20000; % Limit panorama size

%% STEP 1: LOAD AND PREPROCESS ALL IMAGES

[images, gray_images, original_sizes] = loadAndPreprocessImages(config);

%% STEP 2: DETECT FEATURES IN ALL IMAGES WITH SIFT

[points, features] = detectFeaturesAllImages(gray_images, config);

%% STEP 3: MATCH ADJACENT PAIRS

[tforms, match_stats] = matchAdjacentPairs(points, features, config);

%% STEP 4: COMPUTE GLOBAL TRANSFORMATIONS RELATIVE TO CENTER IMAGE

global_tforms = computeGlobalTransformations(tforms, config);

%% STEP 5: CALCULATE GLOBAL BOUNDING BOX

[outputRef, bbox_info] = computeGlobalBoundingBox(images, global_tforms, config);

%% STEP 6: WARP AND BLEND IMAGES

blended_image = warpAndBlendImages(images, global_tforms, outputRef, bbox_info, config);

%% STEP 7: DISPLAY AND SAVE RESULTS

displayAndSaveResults(blended_image, match_stats, bbox_info, config);

%% ========================================================================
%  FUNCTIONS
%% ========================================================================

function [images, gray_images, original_sizes] = loadAndPreprocessImages(config)
    % Load and scale all images from the specified folder
    %
    % Outputs:
    %   images - Cell array of scaled RGB images
    %   gray_images - Cell array of grayscale images
    %   original_sizes - Array storing original image dimensions

    images = cell(config.NUM_IMAGES, 1);
    gray_images = cell(config.NUM_IMAGES, 1);
    original_sizes = zeros(config.NUM_IMAGES, 2);

    for i = 1:config.NUM_IMAGES
        % Load image
        filename = fullfile(config.IMAGE_FOLDER, sprintf('%02d.png', i));

        if ~isfile(filename)
            error('Image file not found: %s', filename);
        end

        img = imread(filename);
        original_sizes(i, :) = [size(img, 1), size(img, 2)];

        % Scale to specified factor to optimize memory and speed
        img = imresize(img, config.SCALE_FACTOR);
        images{i} = img;

        % Convert to grayscale for feature detection
        gray_images{i} = im2gray(img);

    end

end

function [points, features] = detectFeaturesAllImages(gray_images, config)
    % Detect and extract SIFT features from all images
    %
    % Outputs:
    %   points - Cell array of valid feature points
    %   features - Cell array of feature descriptors

    num_images = config.NUM_IMAGES;
    points = cell(num_images, 1);
    features = cell(num_images, 1);

    total_features = 0;

    for i = 1:num_images
        % Detect SIFT features
        detected_points = detectSIFTFeatures(gray_images{i});

        % Extract feature descriptors
        [feat, valid_pts] = extractFeatures(gray_images{i}, detected_points);

        points{i} = valid_pts;
        features{i} = feat;

        total_features = total_features + size(valid_pts, 1);
        fprintf('--> STEP2: Image %02d: %5d features detected\n', i, size(valid_pts, 1));
    end

    fprintf("=============================================================================\n");
end

function [tforms, match_stats] = matchAdjacentPairs(points, features, config)
    % Match features between adjacent image pairs
    %
    % Outputs:
    %   tforms - Cell array of pairwise transformations (i -> i+1)
    %   match_stats - Structure containing matching statistics

    num_pairs = config.NUM_IMAGES - 1;
    tforms = cell(num_pairs, 1);

    match_stats = struct();
    match_stats.num_matches = zeros(num_pairs, 1);
    match_stats.num_inliers = zeros(num_pairs, 1);
    match_stats.inlier_ratio = zeros(num_pairs, 1);

    for i = 1:num_pairs
        fprintf('--> STEP3: Matching images %02d <-> %02d: ', i, i + 1);

        % Match features between adjacent images
        index_pairs = matchFeatures(features{i}, features{i + 1}, 'Method', config.MATCH_METHOD, 'MaxRatio', 0.5, 'Unique', true);

        if size(index_pairs, 1) < 4
            error('Insufficient matches between images %d and %d. Found only %d matches.', ...
                i, i + 1, size(index_pairs, 1));
        end

        matched_points1 = points{i}(index_pairs(:, 1));
        matched_points2 = points{i + 1}(index_pairs(:, 2));

        match_stats.num_matches(i) = size(matched_points1, 1);

        % Estimate projective transformation using RANSAC
        [tform, inlier_idx] = estgeotform2d(matched_points1, matched_points2, ...
            config.TRANSFORM_TYPE, ...
            'MaxNumTrials', 1000000, ...
            'Confidence', 99.999);

        tforms{i} = tform;
        match_stats.num_inliers(i) = sum(inlier_idx);
        match_stats.inlier_ratio(i) = match_stats.num_inliers(i) / match_stats.num_matches(i);

        fprintf('%4d matches, %4d inliers (%.1f%%)\n', ...
            match_stats.num_matches(i), ...
            match_stats.num_inliers(i), ...
            100 * match_stats.inlier_ratio(i));
    end

    fprintf("=============================================================================\n");

end

function global_tforms = computeGlobalTransformations(tforms, config)
    % Compute transformations relative to center image
    %
    % Uses homography propagation to chain transformations:
    %   H_{i->center} = H_{i+1->center} * H_{i->i+1}  (for i < center)
    %   H_{i->center} = H_{i-1->center} * H_{i->i-1}  (for i > center)
    %
    % Output:
    %   global_tforms - Cell array of transformations mapping each image to center

    num_images = config.NUM_IMAGES;
    center_idx = config.CENTER_IMAGE_INDEX;

    global_tforms = cell(num_images, 1);

    % Center image has identity transformation
    global_tforms{center_idx} = affinetform2d(eye(3));
    fprintf('  Image %02d: Identity (center reference)\n', center_idx);

    % Propagate transformations backward (center-1 -> 1)
    % For images to the LEFT of center
    fprintf('  Backward propagation (images %d to 1)...\n', center_idx - 1);

    for i = (center_idx - 1):-1:1
        % tforms{i} maps from image i to image i+1
        % We need H_{i->center} = H_{i+1->center} * H_{i->i+1}

        H_i_to_next = tforms{i}.A; % H_{i->i+1}
        H_next_to_center = global_tforms{i + 1}.A; % H_{i+1->center}

        % Chain the transformations with normalization for numerical stability
        H_i_to_center = H_next_to_center * H_i_to_next;
        H_i_to_center = H_i_to_center / H_i_to_center(3, 3); % Normalize

        % Check condition number
        cond_num = cond(H_i_to_center);

        if cond_num > 1e10
            warning('High condition number (%.2e) for image %d transform', cond_num, i);
        end

        global_tforms{i} = affinetform2d(H_i_to_center);

        fprintf('    Image %02d: cond=%.2e\n', i, cond_num);
    end

    % Propagate transformations forward (center+1 -> end)
    % For images to the RIGHT of center
    fprintf('  Forward propagation (images %d to %d)...\n', center_idx + 1, num_images);

    for i = (center_idx + 1):num_images
        % tforms{i-1} maps from image i-1 to image i
        % We need H_{i->center} = H_{i-1->center} * H_{i->i-1}
        % where H_{i->i-1} = inv(H_{i-1->i}) = inv(tforms{i-1})

        H_prev_to_i = tforms{i - 1}.A; % H_{i-1->i}
        H_i_to_prev = inv(H_prev_to_i); % H_{i->i-1}
        H_prev_to_center = global_tforms{i - 1}.A; % H_{i-1->center}

        % Chain the transformations with normalization
        H_i_to_center = H_prev_to_center * H_i_to_prev;
        H_i_to_center = H_i_to_center / H_i_to_center(3, 3); % Normalize

        % Check condition number
        cond_num = cond(H_i_to_center);

        if cond_num > 1e10
            warning('High condition number (%.2e) for image %d transform', cond_num, i);
        end

        global_tforms{i} = affinetform2d(H_i_to_center);

        fprintf('    Image %02d: cond=%.2e\n', i, cond_num);
    end

end

function [outputRef, bbox_info] = computeGlobalBoundingBox(images, global_tforms, config)
    % Calculate the bounding box containing all warped images
    %
    % Outputs:
    %   outputRef - imref2d object defining the output coordinate system
    %   bbox_info - Structure containing bounding box information

    num_images = config.NUM_IMAGES;

    xlimits_all = zeros(num_images, 2);
    ylimits_all = zeros(num_images, 2);

    for i = 1:num_images
        [xlim, ylim] = outputLimits(global_tforms{i}, ...
            [1 size(images{i}, 2)], [1 size(images{i}, 1)]);
        xlimits_all(i, :) = xlim;
        ylimits_all(i, :) = ylim;
    end

    % Compute overall bounds
    bbox_info = struct();
    bbox_info.xMin = min(xlimits_all(:, 1));
    bbox_info.xMax = max(xlimits_all(:, 2));
    bbox_info.yMin = min(ylimits_all(:, 1));
    bbox_info.yMax = max(ylimits_all(:, 2));

    % Calculate output dimensions
    bbox_info.width = ceil(bbox_info.xMax - bbox_info.xMin);
    bbox_info.height = ceil(bbox_info.yMax - bbox_info.yMin);

    % Limit panorama size to prevent RAM issues
    max_dim = config.MAX_PANORAMA_DIM;

    if bbox_info.width > max_dim || bbox_info.height > max_dim
        scale = min(max_dim / bbox_info.width, max_dim / bbox_info.height);
        fprintf('  WARNING: Limiting panorama size (scale=%.2f) to fit in memory\n', scale);

        % Scale the bounding box
        center_x = (bbox_info.xMin + bbox_info.xMax) / 2;
        center_y = (bbox_info.yMin + bbox_info.yMax) / 2;
        half_w = bbox_info.width * scale / 2;
        half_h = bbox_info.height * scale / 2;

        bbox_info.xMin = center_x - half_w;
        bbox_info.xMax = center_x + half_w;
        bbox_info.yMin = center_y - half_h;
        bbox_info.yMax = center_y + half_h;
        bbox_info.width = ceil(bbox_info.xMax - bbox_info.xMin);
        bbox_info.height = ceil(bbox_info.yMax - bbox_info.yMin);
    end

    % Create output spatial reference
    outputRef = imref2d([bbox_info.height, bbox_info.width], ...
        [bbox_info.xMin bbox_info.xMax], [bbox_info.yMin bbox_info.yMax]);
end

function blended_image = warpAndBlendImages(images, global_tforms, outputRef, bbox_info, config)
    % Warp all images and blend using distance-transform weights
    %
    % Output:
    %   blended_image - Final blended panoramic image

    num_images = config.NUM_IMAGES;
    height = bbox_info.height;
    width = bbox_info.width;
    num_channels = size(images{1}, 3);

    % Use SINGLE precision to save memory (50% reduction)
    weighted_sum = zeros(height, width, num_channels, 'single');
    total_weight = zeros(height, width, 'single');

    fprintf('--> STEP6: Warping and blending images.... wait ;)\n');

    for i = 1:num_images

        try
            % Warp image to panorama coordinate system
            warped_img = imwarp(images{i}, global_tforms{i}, ...
                'OutputView', outputRef);

            % Create mask for valid pixels
            mask = imwarp(true(size(images{i}, 1), size(images{i}, 2)), ...
                global_tforms{i}, 'OutputView', outputRef);

            % Convert to single for blending (saves memory)
            warped_img = im2single(warped_img);

            % Compute distance transform weight (smooth blending at boundaries)
            dist = bwdist(~mask);
            weight = single(dist);

            % Accumulate weighted image
            for c = 1:num_channels
                weighted_sum(:, :, c) = weighted_sum(:, :, c) + weight .* warped_img(:, :, c);
            end

            total_weight = total_weight + weight;

            % Report coverage
            coverage = sum(mask(:)) / (height * width) * 100;
            fprintf('coverage %.1f%%\n', coverage);

        catch ME
            fprintf('SKIPPED (error: %s)\n', ME.message);
        end

        % Clear temporary variables to free memory
        clear warped_img mask dist weight;
    end

    % Normalize by total weight to get final blended image
    fprintf('  Normalizing blended image...\n');
    blended_image = zeros(size(weighted_sum), 'single');
    valid_pixels = total_weight > 0;

    for c = 1:num_channels
        channel = weighted_sum(:, :, c);
        channel(valid_pixels) = channel(valid_pixels) ./ total_weight(valid_pixels);
        blended_image(:, :, c) = channel;
    end

    % Convert back to double for display/save compatibility
    blended_image = double(blended_image);
end

function displayAndSaveResults(blended_image, ~, bbox_info, config)
    % Display final panorama and save to file

    figure('Name', 'Final Panorama - n Images', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1200, 600]);
    imshow(blended_image);
    title(sprintf('Final Panoramic Image (%d Images, %dx%d px, Center: Image %d)', ...
        config.NUM_IMAGES, bbox_info.width, bbox_info.height, config.CENTER_IMAGE_INDEX), ...
        'FontSize', 14);

    imwrite(blended_image, config.OUTPUT_FILENAME, 'Quality', 95);
    fprintf('Panorama saved as: %s\n', config.OUTPUT_FILENAME);
    fprintf('  Panorama dimensions: %d x %d pixels\n', bbox_info.width, bbox_info.height);
end
