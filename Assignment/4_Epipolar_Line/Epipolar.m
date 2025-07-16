clearvars; close all; clc;

addpath("..\0_Funzioni_Utili");

% Carico le calibrazioni che avevo già fatto e le immagini
load Calib_direct_1.mat A
K1 = A;

load Calib_direct_2.mat A
K2 = A;

image1 = imread("Image1.jpg");
image2 = imread("Image2.jpg");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ora inizio ad acquisire i punti corrispondenti nelle 2 imamgini per calcolare la matrice fondamentale %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

numPoints = 8; % perchè servono almeno 8 punti per stimare la matrice fondamentale visto che è una matrice 3x3 con 9 parametri, di cui 1 dato dal fattore di scala e quindi che si può ipotizzare uguale ad 1
ul = []; ur = [];
vl = []; vr = [];
imshow(image1); hold on;
[ul, vl] = click_input_utente(numPoints, ul, vl);
close;
imshow(image2); hold on;
[ur, vr] = click_input_utente(numPoints, ur, vr);
close;

[F, E] = fundamental_AND_essential_matrix(ul, ur, vr, vl, K1, K2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ora inizio a calcolare l'equazione della retta nell'immagine di destra, viene disegnata %
% la retta epipolare associata al punto che si clicca nell'immagine di sinsitra           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ul = []; ur = [];
vl = []; vr = [];

X_epipolar_line = 1:size(image2, 2); % è fatto per trovare un po' di valori delle x che poi associerò alle y nell'equazione della retta

for i = 1:4
    figure(1);
    imshow(image1); hold on;
    [ul, vl] = ginput(1);
    plot(ul, vl, "x", "Color", "red");

    left_P = [ul; vl; 1]; % in coordinate omogenee!
    right_P = F * left_P; % trovo dove si trova il punto left_P nell'immagine di destra, questo grazie alla matrice fondamentale

    % Coefficienti dell'equazione della retta (vengono fuori dalla formula)
    a = F(1, 1) * ul + F(2, 1) * vl + F(3, 1);
    b = F(1, 2) * ul + F(2, 2) * vl + F(3, 2);
    c = F(1, 3) * ul + F(2, 3) * vl + F(3, 3);

    % Calcola i corrispondenti valori di y utilizzando l'equazione della retta
    Y_epipolar_line = (-a * X_epipolar_line - c) / b;

    % Disegna la retta
    figure(2);
    imshow(image2); hold on
    plot(X_epipolar_line, Y_epipolar_line, 'b', 'LineWidth', 2);
    title('Grafico della retta ax + by + c = 0');

end
