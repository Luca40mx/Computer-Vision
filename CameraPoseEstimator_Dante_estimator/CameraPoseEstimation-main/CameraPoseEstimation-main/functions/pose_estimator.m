function [R, T] = pose_estimator(model,checkImageFile, method, testK)
disp('Loading test image...');
checkImg =  imread(checkImageFile);

% sift match descriptors
disp('Computing test image descriptors...');
[fc, dc] = vl_sift(single(rgb2gray(checkImg)));
disp('Matching descriptors with reference image...');
[matches, scores] = vl_ubcmatch(model.d, dc);

[drop, perm] = sort(scores, 'ascend');

toPlot = size(perm,2);
fprintf('Found %i matched points\n', toPlot);
matches = matches(:, perm(1:toPlot));
% scores = scores(perm(1:toPlot));

% x_ref = model.f(1,matches(1,:));
% x_check = fc(1,matches(2,:))+size(model.image,2);
% y_ref = model.f(2,matches(1,:));
% y_check = fc(2,matches(2,:));
% p2D_refMatch = [x_ref', y_ref'];
p2D_check = fc(1:2,matches(2,:))';
p3D_check = model.p3D(matches(1,:),:);

%looad from file

% find best model for matched points
inlier_threshold = floor(length(p2D_check)*0.5);
numIter = 5000;
fprintf('Applying ransac with %i iterations and a threshold of %i inliers...\n', ...
    numIter, inlier_threshold);
[inliers] = ransacPose(p2D_check, p3D_check,testK,numIter,8,inlier_threshold);
numInliers = length(inliers);
fprintf('Best model has %i inliers\n', numInliers);
if  numInliers < 10
    error('Too few matching points to estimate the camera pose')
end

if numInliers < inlier_threshold
    disp('Inliers matching points less than imposed threshold: MODEL COULD BE INACCURATE!')
end

modelPoints = 1:numInliers;
p2D_best = p2D_check(inliers,:);
p3D_best = p3D_check(inliers,:);



disp('Computing exterior parameters');
G = compute_exterior(testK,[eye(3) zeros(3,1)], p2D_best(modelPoints,:)',p3D_best(modelPoints,:)', method);
G %show on command line
plotOnImage(checkImg,p2D_best(modelPoints,:), p3D_best(modelPoints,:), testK, G);
title(strcat('Projection of best model points on test image with ',string(method)));

R = G(1:3,1:3);
T = G(1:3, 4);
end

