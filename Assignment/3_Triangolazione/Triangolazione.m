clc; close all; clearvars;
addpath("..\0_Funzioni_Utili");

% Carico la calibrazione che ho calcolato con il metodo diretto dell'esercizio precedente
% e da qui ci estraggo la matrice di proiezione che si chiama P e la salvo nella variabile P1
load Calib_direct_1.mat P
P1 = P;

load Calib_direct_2.mat P
P2 = P;

image1 = imread("Image1.jpg");
image2 = imread("Image2.jpg");

numPoint = 2; % numero di punti da acquisire in un immagine

%% acquisizione punti sull'immagine di sinistra
figure(1);
imshow(image1);
title("Click 2 points!", "FontSize", 15);
hold on;
x_left = [];
y_left = [];

[x_left, y_left] = click_input_utente(numPoint, x_left, y_left);
close figure 1

u1_left = x_left(1, 1); u2_left = x_left(1, 2); v1_left = y_left(1, 1); v2_left = y_left(1, 2);

%% acquisizione punti sull'immagine di destra
figure(2);
imshow(image2);
title("Click 2 corresponding points W.R.T. the first image!", "FontSize", 15);
hold on;
x_right = [];
y_right = [];

[x_right, y_right] = click_input_utente(numPoint, x_right, y_right);
close figure 2

u1_right = x_right(1, 1); u2_right = x_right(1, 2); v1_right = y_right(1, 1); v2_right = y_right(1, 2);

%% estraggo le righe della matrice prospettica P1 e P2 che poi mi serviranno nella matrice A
% che sarà quella dei termini noti nel sistema lineare
P1_1 = P1(1, :);
P1_2 = P1(2, :);
P1_3 = P1(3, :);
P2_1 = P2(1, :);
P2_2 = P2(2, :);
P2_3 = P2(3, :);

A1 = [P1_1 - u1_left * P1_3;
      P1_2 - v1_left * P1_3;
      P2_1 - u1_right * P2_3;
      P2_2 - v1_right * P2_3];

A2 = [P1_1 - u2_left * P1_3;
      P1_2 - v2_left * P1_3;
      P2_1 - u2_right * P2_3;
      P2_2 - v2_right * P2_3];

[~, ~, V1] = svd(A1);
[~, ~, V2] = svd(A2);
M1 = V1(:, end); % prendo ultima colonna della matrice V perchè dal un corollario del teorema del SVD decomposition --> il vettore nullo che minimizza il risultato si trova nell'ultima colonna della amtrice V
M2 = V2(:, end);

M1_norm = M1(1:3, 1) / M1(4, 1); % normalizzo il risultato solo perchè ha 4 componenti e io ne voglio solamente 3 (x,y,z) relative al punto reale associato alle coordinate in pixel che ho estratto in precedenza
M2_norm = M2(1:3, 1) / M2(4, 1);

distanza = sqrt((M1_norm(1, 1) - M2_norm(1, 1)) ^ 2 + (M1_norm(2, 1) - M2_norm(2, 1)) ^ 2 + (M1_norm(3, 1) - M2_norm(3, 1)) ^ 2);

% ora voglio disegnare questa distanza sulla prima immagine
imshow(image1);
hold on;
plot(u1_left, v1_left, "x", "Color", "red", "LineWidth", 2, "MarkerSize", 20); hold on
plot(u2_left, v2_left, "x", "Color", "green", "LineWidth", 2, "MarkerSize", 20); hold on
line([u1_left, u2_left], [v1_left, v2_left], "Color", "black", "LineWidth", 4);
testo = ['Distanza = ' num2str(distanza, '%.2f')];
posizione_x_testo = (u1_left + u1_left) / 2 +10;
posizione_y_testo = (v1_left + v2_left) / 2 + 10;
text(posizione_x_testo, posizione_y_testo, testo, 'Color', 'r', 'FontSize', 20);
