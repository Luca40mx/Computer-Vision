clc; clearvars; close all;

% Caricamento informazioni camera
load('imgInfo.mat');

% Immagine
% img = imread('lab1.png');
img = imread('cav.jpg');

m = imgInfo.punti2DImg;
m = [m ones(size(m, 1), 1)]';
M = imgInfo.punti3DImg;
M = [M ones(size(M, 1), 1)]';
K = imgInfo.K;

[s, R, t] = fiore(K, m(:, 1:100), M(:, 1:100));
P = K * [R t];

m_reproj = project(P, M);

figure(1);
imshow(img);
hold on;
plot(m(1, :), m(2, :), 'r.');
plot(m_reproj(1, :), m_reproj(2, :), 'go');

function [s, R, t] = fiore(K, m, M)

    n = size(m, 2);
    m = m ./ m(3, :);
    M = M ./ M(4, :);

    [~, ~, V] = svd(M);
    r = rank(M);
    min_n = ceil((3 * r - 1) / 2);

    if min_n > n
        error('Insufficient number of points! Needed %d, provided %d', min_n, n);
    end

    Vr = V(:, (r + 1):end);

    D = zeros(3 * n, n);

    for i = 1:n
        D((3 * i - 2):(3 * i), i) = m(:, i);
    end

    [~, ~, V] = svd(kron(Vr', inv(K)) * D, 'econ');
    Z = V(:, end);

    X = Z' .* (K \ m);
    [s, R, t] = absolute_orientation(X, M(1:3, :));

end

function [m] = project(P, M)

    if ~ismatrix(M) || size(M, 1) ~= 4
        error('Scene points must be given as homogeneous coordinates (M must be a 4-by-n matrix)');
    end

    if ~ismatrix(M) || ~isequal(size(P), [3 4])
        error('Camera is not valid (P must be a 3-by-4 matrix)');
    end

    m = P * M;
    % m is not in uniform coordinates, so we scale it by the m.z value
    m = m ./ m(3, :);
    m(isnan(m)) = 1; % removes NaN given by 0/0 and Inf/Inf

end

function [s, R, t] = absolute_orientation(X, Y)
    %ABSOLUTE_ALIGNMENT Fits Y to X using the least squared distances approach
    %   Resolves the problem of min_{R,t}sum_{i=1}^{N}norm{X_i - s(RY_i +
    %   t)}^2. Given two DxN matrices, X for model and Y for data, ideally
    %   representing D-dimensional points X_i and Y_i disposed by column, tries
    %   to fit as good as possible Y to X, applying a rototranslation (R,t) and
    %   scaling by s.
    if size(X) ~= size(Y)
        error('Dimensions do not match');
    end

    if size(X, 1) < size(X, 2)
        X = X';
        Y = Y';
    end

    % discard nan values
    i = find(~isnan(X));
    X = reshape(X(i), length(X(i)) / 3, 3)';
    Y = reshape(Y(i), length(Y(i)) / 3, 3)';

    % center the clouds
    centroid_X = mean(X, 2);
    centroid_Y = mean(Y, 2);
    cX = X - centroid_X;
    cY = Y - centroid_Y;

    % extract the parameters
    s = norm(cX(:, 1)) / norm(cY(:, 1));
    [U, ~, Vt] = svd(cY * cX');
    R = (U * diag([1 1 det(U * Vt')]) * Vt')';
    t = (1 / s) * centroid_X - R * centroid_Y;
end
