clc; clearvars; close all;

% Load calibration points
load('calibrationChessboard.mat');

% Load chessboard image
img = imread('chessboard.jpg');

figure('Name', 'Smooth Chessboard Points Animation', 'Position', [100, 100, 800, 600]);

imshow(img);
hold on;
title('Animation', 'FontSize', 14);

h_ball = plot(points(1, 1), points(1, 2), 'o', 'MarkerSize', 10, 'Color', 'red', 'MarkerFaceColor', 'red');

trail_x = [];
trail_y = [];
h_trail = plot(trail_x, trail_y, 'g-', 'LineWidth', 2);

steps_between_points = 20;
pause_time = 0.05;
num_cycles = 1; % Number of complete cycles

for cycle = 1:num_cycles

    trail_x = [];
    trail_y = [];

    for i = 1:size(points, 1)

        if i == size(points, 1)
            next_point = points(1, :);
        else
            next_point = points(i + 1, :);
        end

        current_point = points(i, :);

        x_interp = linspace(current_point(1), next_point(1), steps_between_points);
        y_interp = linspace(current_point(2), next_point(2), steps_between_points);

        for step = 1:steps_between_points
            set(h_ball, 'XData', x_interp(step), 'YData', y_interp(step));

            trail_x = [trail_x, x_interp(step)]; %#ok<*AGROW>
            trail_y = [trail_y, y_interp(step)];

            max_trail_length = 100;

            if length(trail_x) > max_trail_length
                trail_x = trail_x(end - max_trail_length + 1:end);
                trail_y = trail_y(end - max_trail_length + 1:end);
            end

            set(h_trail, 'XData', trail_x, 'YData', trail_y);

            pause(pause_time);
        end

    end

    if cycle < num_cycles
        pause(0.5);
        delete(h_text);
        set(h_trail, 'XData', [], 'YData', []);
    end

end
