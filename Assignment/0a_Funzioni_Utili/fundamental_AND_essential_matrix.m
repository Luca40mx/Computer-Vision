% Questa funzione serve per calcolare la MATRICE FONDAMENTALE e la MATRICE ESSENZIALE.

% come INPUT prende le coordinate in pixel dei punti che l'utente clicca sull'immagine di sinistra
% e i punti CORRISPONDENTI nell'immagine di destra. Devo estrarne almeno 8 perchè la matrice fondamentale è una 3x3.
% Inoltre ha come input i parametri interni delle 2 camere che serviranno per calcolare la matrice essenziale.

% come OUTPUT ti ritorna prima la matrice fondamentale e poi la matrice essenziale.

function [F, E] = fundamental_AND_essential_matrix(ul, ur, vr, vl, K1, K2)
    numPoints = 8; % perchè servono almeno 8 punti per stimare la matrice fondamentale visto che è una matrice 3x3 con 9 parametri, di cui 1 dato dal fattore di scala e quindi che si può ipotizzare uguale ad 1
    A = [];

    for i = 1:numPoints
        A = [A; ul(i) * ur(i), ul(i) * vr(i), ul(i), vl(i) * ur(i), vl(i) * vr(i) vl(i) ur(i) vr(i) 1]; %#ok<*AGROW>
    end

    [~, ~, V] = svd(A);

    F = V(:, end); % ultima colonna di V per un corollario del teorema SVD.

    F = [F(1, 1) F(2, 1) F(3, 1); % fundamental matrix
         F(4, 1) F(5, 1) F(6, 1);
         F(7, 1) F(8, 1) F(9, 1)];

    E = K1.' * F * K2; % essential matrix
end
