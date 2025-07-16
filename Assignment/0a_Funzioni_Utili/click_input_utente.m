% Questa funzione in modo molto elementare estrae le coordinate in pixel
% dall'immagine caricata in quel momento dell'esecuzione del programma
% principale.

% Ha come INPUT il numero di punti che devi acquisire e 2 array che
% verranno riempiti rispettivamente con le coordinate u e v dei pixel del
% punto cliccato.

% Ha come OUTPUT un array con all'interno le coordinate appena cliccate
% prima tutte le x e poi tutte le y.

%% Ciao Luca del futuro, se stai leggendo questo commento capirai quanto facevi schifo a scrivere codice ;)
%% siamo a ottobre del 2023, good luck with your exams!

function [u, v] = click_input_utente(numPoint, u, v)

    for i = 1:numPoint
        [x, y] = ginput(1);

        u(i) = x; %#ok<*SAGROW>
        v(i) = y;
        plot(u, v, "x", "Color", "red", "MarkerSize", 20, "LineWidth", 2);
    end

end
