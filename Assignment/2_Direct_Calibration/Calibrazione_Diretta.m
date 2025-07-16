clearvars
close
clc

% carica immagine del cubo
calib_img = imread("cubo_rubik.jpg");
image = rgb2gray(calib_img);

% Visualizza immagine del cubo
imshow(image);
title("Click 6 points!")
hold on;

% Punti minimi da indicare per poter trovare tutte le incognite nella matrice P
numPoints = 6;

% ciclo for che serve solo a fare indicare all'utente i punti sull'immagine per estrarne le coordinate in pixel e associarle al rispettivo punto 3D noto
for i = 1:numPoints
    [x, y] = ginput(1);

    pixel_x(i) = x; %#ok<*SAGROW>
    pixel_y(i) = y;
    plot(x, y, "x", "Color", "r", "MarkerSize", 20)
end

close figure 1

% dimensioni reali del cubo
real_x = [0; 5.5; 5.5; 0; 5.5; 5.5];
real_y = [0; 0; 5.5; 5.5; 0; 5.5];
real_z = [5.5; 5.5; 5.5; 5.5; 0; 0];

% Creazione dei simbolici per capire meglio le formule che ne escono
syms u1 u2 u3 u4 u5 u6
syms v1 v2 v3 v4 v5 v6
syms X1 X2 X3 X4 X5 X6
syms Y1 Y2 Y3 Y4 Y5 Y6
syms Z1 Z2 Z3 Z4 Z5 Z6
syms p11 p12 p13 p14 p21 p22 p23 p24 p31 p32 p33 p34

u = [u1 u2 u3 u4 u5 u6];
v = [v1 v2 v3 v4 v5 v6];
X = [X1 X2 X3 X4 X5 X6];
Y = [Y1 Y2 Y3 Y4 Y5 Y6];
Z = [Z1 Z2 Z3 Z4 Z5 Z6];
p = [p11 p12 p13 p14; p21 p22 p23 p24; p31 p32 p33 p34];
p = reshape(p.', 12, 1); % per metterlo in verticale, perchè una volta che dovrai risolvere il sistema A * p = 0, p deve essere un vettore 12x1 per poter fare la moltiplcazione
A = [];

% crea la matrice dei termini noti nel sistema, ha questo pattern ripetitivo a seguito dei calcoli che vengono fatti nella parte di teoria
for i = 1:numPoints

    R1 = [X(i) Y(i) Z(i) 1 0 0 0 0 -u(i) * X(i) -u(i) * Y(i) -u(i) * Z(i) -u(i)];
    R2 = [0 0 0 0 X(i) Y(i) Z(i) 1 -v(i) * X(i) -v(i) * Y(i) -v(i) * Z(i) -v(i)];
    A = [A; R1; R2]; %#ok<*AGROW>
end

% metto i valori reali dei simbolici utilizzati prima
A = subs(A, [X Y Z u v], [[0, 5.5, 5.5, 0, 5.5, 5.5] [0 0 5.5 5.5 0 5.5] [5.5 5.5 5.5 5.5 0 0] pixel_x pixel_y]);

% faccio la SVD sulla matrice A trovata sopra per poter estrarre la matrice V dalla quale estraggo l'ultima colonna, che contiene i coefficienti cercati per la matrice p
[U, D, V] = svd(A);

P = V(:, end); % estrae l'ultima colonna dalla matrice V, che è quella che ci interessa per trovare i valori della matrice p = [p11 p12 p13 p14; p21 p22 p23 p24; p31 p32 p33 p34]
P = round(P, 4);
P = [P(1) P(2) P(3) P(4); P(5) P(6) P(7) P(8); P(9) P(10) P(11) P(12)]; %   QUESTA E' LA MATRICE DI PROIEZIONE FINALE CORRETTA!

% ora trovo i parametri intrinseci ed estrinseci della camera.
% i parametri interni e la rotazione li trovo con la decomposizione QR che mi restituisce una matrice triangolare superiore e una matrice ortogonale rispettivamente i parametri interni e la rotazione
[R, K] = qr(P(1:3, 1:3));
internal_K = round(K, 4);
rotation_R = round(R, 4);

% per trovare anche la traslazione della camera faccio una formula inversa per ricavare t
tmp = inv(K) * P(:, end); %#ok<MINV>
translation_t = round(tmp, 4);

save("..\0_Internal_parameters_AND_projection_matrix\internal_parameters.mat", "internal_K", "rotation_R", "translation_t")
