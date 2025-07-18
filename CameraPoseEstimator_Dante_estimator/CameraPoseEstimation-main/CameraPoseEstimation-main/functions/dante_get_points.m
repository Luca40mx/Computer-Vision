function [K, R, T, p2D, p3D] = dante_get_points(zephyrPlyFile,visibilityPointFile, xmpFile)
% Analyse Zephyr outputs:
%
% Read cloud of points, and project points to 2D images (using information
% on visibility)
%
directory_ref = '.\dante\'; %#ok<NASGU>
%Read and show sparse point clouds: (there are many noisy points, it is good to subsample the cloud) 
[Points] = plyread(zephyrPlyFile);
X=[Points.vertex.x Points.vertex.y Points.vertex.z];
fid=fopen(visibilityPointFile, 'r');
string=fgetl(fid);
name_view=string(end-11:end);
% 
npoint=str2double(fgetl(fid));
disp(['Processing file: ' name_view ' with points ' num2str(npoint)]);
VisPoints=zeros(npoint,3);
for p=1:npoint
    VisPoints (p,:)=str2double(fgetl(fid));
    %In matlab array starts from 1:
    VisPoints(p, 1)= VisPoints(p, 1)+1;
end
% Read visible points
Xvis=X(VisPoints(:,1),:);
p2D = VisPoints(:,2:3);
p3D = Xvis;

[K, R, T] = read_xmp(xmpFile);
end






