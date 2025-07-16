%% Rettificazione 
% Lo scopo è trovare due nuove matrici MPP (una per ogni camera), ottenute
% ruotando le matrici originali attorno ai loro centri ottici finchè i
% piani focali non diventano complanari. Facendo cosi le rette epipolari
% risulteranno parallele e orizzontali. 

clear all
close all
clc

%Caricamento dati della calibrazione
stereo = load('calib/Calib_Results_stereo.mat');
left = load('calib/Calib_Results_left.mat');
right = load('calib/Calib_Results_right.mat');

img1=imread('Scacchiera/left01.jpg');
img2=imread('Scacchiera/right01.jpg');

%Aggiorno il path:
%--
%addpath(genpath('../TOOLBOX_calib'));

% Primo passo: proiezione dei punti stereo nelle due camere

data3d=stereo.X_left_1; % punti 3d di partenza

% camera 1: sistema di riferimento centrato su questa camera
PPM_left = stereo.KK_left*[1 0 0 0;0 1 0 0;0 0 1 0]*eye(4);

%devo spostare i punti:
Rc_left = rodrigues(stereo.omc_left_1);
G_c = [Rc_left, stereo.Tc_left_1;0 0 0 1];
X_c = G_c * [data3d; ones(1,size(data3d,2))];
M_l=PPM_left*X_c;
xl=M_l(1,:)./M_l(3,:);
yl=M_l(2,:)./M_l(3,:);

% camera 2 del setup stereo
G_r=[stereo.R stereo.T;0 0 0 1];
PPM_right = stereo.KK_right*[1 0 0 0;0 1 0 0;0 0 1 0]*G_r;
M_r=PPM_right*X_c;
xr=M_r(1,:)./M_r(3,:);
yr=M_r(2,:)./M_r(3,:);

% scomposizione delle PPM
Q1=PPM_left(:,1:3);
Q2=PPM_right(:,1:3);
q1=PPM_left(:,4);
q2=PPM_right(:,4);


% funzione art: fattorizzazione della matrice MPP nelle matrici dei
% parametri intrinseci, di rotazione e traslazione
% Non è necessario visto che sopra ho già tutti i dati!
[K1,R1,t1] = art(PPM_left);
[K2,R2,t2] = art(PPM_right);

% Ottendo i centri ottici delle due camere C=-inv(Q1)*q
% essendo i sistema centrato nella prima camera il suo centro ottico avrà
% coordinate 3d=[0 0 0]' cartesiane
c1 = - inv(Q1)*q1;
c2= - inv(Q2)*q2;

% Costruisco la matrice di rotazione R=[r1' r2' r3']'
% r1: nuovo asse X, dovrè essere parallelo alla linea di base
v1 = (c2-c1);
% Pongo un versore uguale alla vecchia asse Z
k=R1(3,:);
% r2: nuovo asse Y, ortogonale all'asse X e al vecchio asse Z
v2 = cross(k',v1);
% r3: nuovo asse Z, ortogonale all'asse X e Y
v3 = cross(v1,v2);

% Normalizzo e compongo R 
R = [v1'/norm(v1); v2'/norm(v2); v3'/norm(v3)];

% Compongo le nuove MPP, pongo K (matrice dei parametri intrinseci) uguale
% a quella della camera 1
Pn1 = K1 * [R -R*c1 ];
Pn2 = K1 * [R -R*c2 ];

% Ottengo la matrice di trasfromazione T=Qn*inv(Qo), dove Qo si riferisce
% alla vecchia MPP e Qn a quella nuova. T è una matrice 3x3 che permette
% una trasformazione lineare tra spazi proiettivi (è una matrice di 
% omografia)

T1=Pn1(:,1:3)*inv(Q1);
T2=Pn2(:,1:3)*inv(Q2);

% Calcolo le nuove proiezioni dei punti rettificati
% X_c: coordinate omogenee dei punti 3d nel sistema camera 1

Xl_new = T1*M_l;
Xr_new = T2*M_r;

% Passo alle coordinate cartesiane
Xl(1,:)=Xl_new(1,:)./Xl_new(3,:);
Xl(2,:)=Xl_new(2,:)./Xl_new(3,:);
Xr(1,:)=Xr_new(1,:)./Xr_new(3,:);
Xr(2,:)=Xr_new(2,:)./Xr_new(3,:);

% immagine rettificate ottenute dalle funzioni del toolbox
% img1_rec=imread('./img_left_rectifield/left_rectified01.bmp');
% img2_rec=imread('./img_right_rectifield/right_rectified01.bmp');

img1_rec = imwarp(img1,T1);
img2_rec = imwarp(img2,T2);



% Visualizzazione risultati
figure(1);
subplot(221); imshow(img1); hold on
scatter(xl,yl,'b*');
subplot(222); imshow(img2); hold on
scatter(xr,yr,'r*');

subplot(223); imshow(uint8(img1_rec)); hold on
scatter(Xl(1,:),Xl(2,:),'b*');
subplot(224); imshow(uint8(img2_rec)); hold on
scatter(Xr(1,:),Xr(2,:),'r*');

% Proiezione delle rette epipolari
% Epipolo
e_r = Pn2*[-inv(Pn1(:,1:3))*Pn1(:,4);1];
% Matrice fondamentale
F= [0 -e_r(3) e_r(2); e_r(3) 0 -e_r(1);-e_r(2) e_r(1) 0];
F=F*Pn2(:,1:3)*inv(Pn1(:,1:3));

% Selezione e rappresentazione di alcuni punti a caso della scacchiera
point=[1 10 20 30];
figure(2);
subplot(121); imshow(uint8(img1_rec)); hold on
scatter(Xl(1,point),Xl(2,point),'r*');
subplot(122); imshow(uint8(img2_rec)); hold on

% Per ogni punto della prima immagine calcolo la retta epipolare nella 
% seconda immagine
x=1:size(img2_rec,2);
for i=point    
    % calcolo la retta epipolare da F*m, au+bv+c=0
    coeff_retta=F*Xl_new(:,i);
    y=-x*(coeff_retta(1)/coeff_retta(2))-coeff_retta(3)/coeff_retta(2);
    
    scatter(Xr(1,i),Xr(2,i),'r*');
    plot(x,y);
end

% Per effetto della rettificazione la retta risultante è orrizontale,
% infatti i valori delle y della retta sono tutti uguali.
