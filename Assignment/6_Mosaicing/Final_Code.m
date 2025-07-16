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
[Image1, Image2, exposure_value1, exposure_value2, media_exposure_value] = luminosita(I1, I2, image1Size, image2Size);

%Catch the feature points found in two pictures through sift
[points1, points2] = SIFT_Detection(Image1, Image2);

% Associazione dei punti caratteristici
[match_img1_points, match_img2_points] = CorrespondingPoints(points1, Image1, points2, Image2, I1, I2);

% Ransac
tform = RANSAC_Algorithm(match_img1_points, match_img2_points, I1, I2);

% converto Immagine 1 in 2 con H
[Rifermento_fisso, Immagine_trasf_con_H, sys_rif_Immagine_trasf_con_H] = Homography(I2, I1, tform);

% Memorizza le coordinate Mondo
WL = WorldCoordinates(sys_rif_Immagine_trasf_con_H, Rifermento_fisso);

% copiato
final = BOHHHHHHHHHHHHHHHHHHHHHHHHHH(WL, Immagine_trasf_con_H, media_exposure_value, exposure_value1, I2, exposure_value2, image1Size, tform, Image2);

% final
final_photo = uint8(final * 255);
figure();
imshow(final_photo);

function [Image1, Image2, exposure_value1, exposure_value2, media_exposure_value] = luminosita(I1, I2, image1Size, image2Size)
    % funzione per passare da immagini a colori ad immagini in scala di grigi (Image1 e Image2),
    % ritorna anche i valori di esposizione per ciascuna immagine (exposure_value1 e exposure_value2)
    % e infine il valore medio dei valori di esposizione (media_exposure_value) che serviranno dopo.

    Image1 = rgb2gray(I1);
    Image2 = rgb2gray(I2);
    sum_1 = sum(im2double(Image1), "all");
    sum_2 = sum(im2double(Image2), "all");
    exposure_value1 = sum_1 / (image1Size(1) * image1Size(2));
    exposure_value2 = sum_2 / (image2Size(1) * image2Size(2));
    media_exposure_value = (exposure_value1 + exposure_value2) / 2;
end

function [points1, points2] = SIFT_Detection(Image1, Image2)
    points1 = detectSIFTFeatures(Image1);
    points2 = detectSIFTFeatures(Image2);
end

function [match_img1_points, match_img2_points] = CorrespondingPoints(points1, Image1, points2, Image2, I1, I2)
    [Image1Features, points1] = extractFeatures(Image1, points1);
    [Image2Features, points2] = extractFeatures(Image2, points2);
    boxPairs = matchFeatures(Image1Features, Image2Features);
    match_img1_points = points1(boxPairs(:, 1));
    match_img2_points = points2(boxPairs(:, 2));
    figure();
    showMatchedFeatures(I1, I2, match_img1_points, match_img2_points, 'montage') % mostra rette che uniscono i punti corrispondenti tra le 2 immagini
end

function tform = RANSAC_Algorithm(match_img1_points, match_img2_points, I1, I2)
    % tramite RANSAC questa funzione elimina gli outliers e tiene solo gli
    % inliers tracciandone le rette per far vedere quanti punti inutili che
    % SIFT aveva trovato.
    [tform, inlierIdx] = estgeotform2d(match_img1_points, match_img2_points, "projective");
    inliers_img1_points = match_img1_points(inlierIdx, :);
    inliers_img2_points = match_img2_points(inlierIdx, :);
    figure()
    showMatchedFeatures(I1, I2, inliers_img1_points, inliers_img2_points, 'montage') % disegna le nuove rette
end

function [Rifermento_fisso, Immagine_trasf_con_H, sys_rif_Immagine_trasf_con_H] = Homography(I2, I1, tform)
    % funzione che calcola l'omografia tra le 2 immagini e le sovrappone tra
    % loro con la funzione imshowpair (sovrapposizione molto visibile)
    Rifermento_fisso = imref2d(size(I2));
    [Immagine_trasf_con_H, sys_rif_Immagine_trasf_con_H] = imwarp(I1, tform);
    figure()
    imshowpair(I2, Rifermento_fisso, Immagine_trasf_con_H, sys_rif_Immagine_trasf_con_H, "blend", "Scaling", "joint");
end

function WL = WorldCoordinates(sys_rif_Immagine_trasf_con_H, Rifermento_fisso)
    WL = [sys_rif_Immagine_trasf_con_H.XWorldLimits; Rifermento_fisso.XWorldLimits];
    WL(:, :, 2) = [sys_rif_Immagine_trasf_con_H.YWorldLimits; Rifermento_fisso.YWorldLimits]; % (I1 I2, start end, x y)

    for i = 1:2

        for j = 1:2

            for k = 1:2

                if (WL(i, j, k) < 0 && mod(WL(i, j, k), 1) == 0.5)
                    WL(i, j, k) = round(WL(i, j, k)) + 1;
                    continue;
                end

                WL(i, j, k) = round(WL(i, j, k));
            end

        end

    end

end

function final = BOHHHHHHHHHHHHHHHHHHHHHHHHHH(WL, Immagine_trasf_con_H, media_exposure_value, exposure_value1, I2, exposure_value2, image1Size, tform, Image2)
    tmp = min(WL, [], 1);
    tmp = min(tmp, [], 2);
    WL = WL(:, :, :) - tmp;

    tmp = max(WL, [], 1);
    tmp = max(tmp, [], 2);
    rf = tmp(1, 1, 2);
    cf = tmp(1, 1, 1);
    final = zeros(rf, cf, 3);
    coo_I1 = WL(1, :, :);
    m_1 = im2double(Immagine_trasf_con_H);
    m_1 = m_1 * media_exposure_value / exposure_value1;

    for i = 1:coo_I1(1, 2, 2) - coo_I1(1, 1, 2)

        for j = 1:coo_I1(1, 2, 1) - coo_I1(1, 1, 1)
            final(coo_I1(1, 1, 2) + i, coo_I1(1, 1, 1) + j, :) = m_1(i, j, :);
        end

    end

    coo_I2 = WL(2, :, :);
    m_2 = im2double(I2);
    m_2 = m_2 * media_exposure_value / exposure_value2;

    bound_m1 = zeros(image1Size);
    bound_m1(1, :) = 1; bound_m1(image1Size(1), :) = 1;
    bound_m1(:, 1) = 1; bound_m1(:, image1Size(2)) = 1;
    trans_bound_m1 = imwarp(bound_m1, tform);
    bound_dist1 = bwdist(trans_bound_m1);

    bound_m2 = zeros(size(Image2));
    bound_m2(1, :) = 1; bound_m2(image1Size(1), :) = 1;
    bound_m2(:, 1) = 1; bound_m2(:, image1Size(2)) = 1;
    bound_dist2 = bwdist(bound_m2);

    for i = 1:coo_I2(1, 2, 2) - coo_I2(1, 1, 2)

        for j = 1:coo_I2(1, 2, 1) - coo_I2(1, 1, 1)

            if final(coo_I2(1, 1, 2) + i, coo_I2(1, 1, 1) + j, :) == [0, 0, 0] %#ok<BDSCA>
                final(coo_I2(1, 1, 2) + i, coo_I2(1, 1, 1) + j, :) = m_2(i, j, :);
            else
                i_I1 = coo_I2(1, 1, 2) + i - coo_I1(1, 1, 2);
                j_I1 = coo_I2(1, 1, 1) + j - coo_I1(1, 1, 1);
                w_1 = bound_dist1(i_I1, j_I1);
                w_2 = bound_dist2(i, j);

                final(coo_I2(1, 1, 2) + i, coo_I2(1, 1, 1) + j, :) = (final(coo_I2(1, 1, 2) + i, coo_I2(1, 1, 1) + j, :) * w_1 + m_2(i, j, :) * w_2) / (w_1 + w_2);
            end

        end

    end

end
