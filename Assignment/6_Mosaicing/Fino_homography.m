close all
clearvars
clc

% Read Pictures
I1 = imread("Left_Image.jpg");
I2 = imread("Right_Image.jpg");
image1Size = size(I1);
image2Size = size(I2);
imshow([I1, I2])

% Luminosita
[Image1, Image2, exposure_value1, exposure_value2, mean_expo_val] = luminosita(I1, I2, image1Size, image2Size);

%Catch the feature points found in two pictures through sift
[points1, points2] = SIFT_Detection(Image1, Image2);

% Associazione dei punti caratteristici
[matchedimg1Points, matchedimg2Points] = CorrespondingPoints(points1, Image1, points2, Image2, I1, I2);

% Ransac
tform = RANSAC_Algorithm(matchedimg1Points, matchedimg2Points, I1, I2);

% converto Immagine 1 in 2 con H
[Rfixed, registered1, Rregistered1] = Homography(I2, I1, tform);


function [Image1, Image2, exposure_value1, exposure_value2, mean_expo_val] = luminosita(I1, I2, image1Size, image2Size)
    % funzione per passare da immagini a colori ad immagini in scala di grigi (Image1 e Image2),
    % ritorna anche i valori di esposizione per ciascuna immagine (exposure_value1 e exposure_value2)
    % e infine il valore medio dei valori di esposizione (mean_expo_val) che serviranno dopo.

    Image1 = rgb2gray(I1);
    Image2 = rgb2gray(I2);
    sum_1 = sum(im2double(Image1), "all");
    sum_2 = sum(im2double(Image2), "all");
    exposure_value1 = sum_1 / (image1Size(1) * image1Size(2));
    exposure_value2 = sum_2 / (image2Size(1) * image2Size(2));
    mean_expo_val = (exposure_value1 + exposure_value2) / 2;
end

function [points1, points2] = SIFT_Detection(Image1, Image2)
    points1 = detectSIFTFeatures(Image1);
    points2 = detectSIFTFeatures(Image2);
end

function [matchedimg1Points, matchedimg2Points] = CorrespondingPoints(points1, Image1, points2, Image2, I1, I2)
    [Image1Features, points1] = extractFeatures(Image1, points1);
    [Image2Features, points2] = extractFeatures(Image2, points2);
    boxPairs = matchFeatures(Image1Features, Image2Features);
    matchedimg1Points = points1(boxPairs(:, 1));
    matchedimg2Points = points2(boxPairs(:, 2));
    figure();
    showMatchedFeatures(I1, I2, matchedimg1Points, matchedimg2Points, 'montage') % mostra rette che uniscono i punti corrispondenti tra le 2 immagini
end

function tform = RANSAC_Algorithm(matchedimg1Points, matchedimg2Points, I1, I2)
    % tramite RANSAC questa funzione elimina gli outliers e tiene solo gli
    % inliers tracciandone le rette per far vedere quanti punti inutili che
    % SIFT aveva trovato.
    [tform, inlierIdx] = estgeotform2d(matchedimg1Points, matchedimg2Points, "projective");
    inlierimg1Points = matchedimg1Points(inlierIdx, :);
    inlierimg2Points = matchedimg2Points(inlierIdx, :);
    figure()
    showMatchedFeatures(I1, I2, inlierimg1Points, inlierimg2Points, 'montage') % disegna le nuove rette
end

function [Rfixed, registered1, Rregistered1] = Homography(I2, I1, tform)
    % funzione che calcola l'omografia tra le 2 immagini e le sovrappone tra
    % loro con la funzione imshowpair (sovrapposizione molto visibile)
    Rfixed = imref2d(size(I2));
    [registered1, Rregistered1] = imwarp(I1, tform);
    figure()
    imshowpair(I2, Rfixed, registered1, Rregistered1, "blend", "Scaling", "joint");

end