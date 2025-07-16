clc; clearvars; close all;

function H_matrix = compute_homography(xs, ys, xd, yd)

    syms h11 h12 h13 h21 h22 h23 h31 h32 h33 real
    syms xd1 yd1 xd2 yd2 xd3 yd3 xd4 yd4 real % points destination image
    syms xs1 ys1 xs2 ys2 xs3 ys3 xs4 ys4 real % points source image (the one that we will rotate/translate to match the reference system of the destination image)

    homoMatrix = [h11 h12 h13 h21 h22 h23 h31 h32 1]; % 8 unknows --> at least 4 points! more is better

    A = [
         xs1, ys1, 1, 0, 0, 0, -xd1 * xs1, -xd1 * ys1, -xd1;
         0, 0, 0, xs1, ys1, 1, -yd1 * xs1, -yd1 * ys1, -yd1;
         xs2, ys2, 1, 0, 0, 0, -xd2 * xs2, -xd2 * ys2, -xd2;
         0, 0, 0, xs2, ys2, 1, -yd2 * xs2, -yd2 * ys2, -yd2;
         xs3, ys3, 1, 0, 0, 0, -xd3 * xs3, -xd3 * ys3, -xd3;
         0, 0, 0, xs3, ys3, 1, -yd3 * xs3, -yd3 * ys3, -yd3;
         xs4, ys4, 1, 0, 0, 0, -xd4 * xs4, -xd4 * ys4, -xd4;
         0, 0, 0, xs4, ys4, 1, -yd4 * xs4, -yd4 * ys4, -yd4;
         ];

    % assume(norm(homoMatrix) ^ 2 == 1)

    solution = solve(A * homoMatrix' == 0, [h11 h12 h13 h21 h22 h23 h31 h32]);

    xs = xs(1:4);
    ys = ys(1:4);
    xd = xd(1:4);
    yd = yd(1:4);

    [h11, h12, h13, h21, h22, h23, h31, h32] = deal(solution.h11, solution.h12, solution.h13, solution.h21, solution.h22, solution.h23, solution.h31, solution.h32);
    H_matrix = [h11, h12, h13; h21, h22, h23; h31, h32, 1];
    H_matrix = subs(H_matrix, [xd1 yd1 xd2 yd2 xd3 yd3 xd4 yd4 xs1 ys1 xs2 ys2 xs3 ys3 xs4 ys4], [xd(1) yd(1) xd(2) yd(2) xd(3) yd(3) xd(4) yd(4) xs(1) ys(1) xs(2) ys(2) xs(3) ys(3) xs(4) ys(4)]);

end
