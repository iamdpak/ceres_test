clear;
close all;
%% from points
radius = 15;
center = [0,0,0];
fpts = [];

for ang=0:0.01:2*pi
    xyz = [center(1) + radius*cos( ang ), center(2) + radius*sin( ang ),0];
       
       fpts = [fpts;xyz];
end

figure;
scatter3(fpts(:,1),fpts(:,2),fpts(:,3),5,'filled','r');
% write into a file
orgPts = fopen('from_pts.txt','w');
fprintf(orgPts,'%f %f %f\n',transpose(fpts));
fclose(orgPts);

%% to points

% choose S R and T 
rx = 0;
ry = 90;
rz = 0;
eul = [deg2rad(rx) deg2rad(ry) deg2rad(rz)];
rotmZYX = eul2rotm(eul);
sl = 1.1;
trl = [0; 0; 0];

smat = [sl 0 0 0; 0 sl 0 0 ; 0 0 sl 0; 0 0 0 1];
tmat = [rotmZYX trl];
tmat = [tmat ; 0 0 0 1];
tmat = smat * tmat;

tpts=[];
for idx=1:length(fpts)
    pt = [transpose(fpts(idx,:)); 1];
    tpt = tmat * pt;
    tpts = [tpts; transpose(tpt)];
end
tpts(:,4) = [];
hold on;
scatter3(tpts(:,1),tpts(:,2),tpts(:,3),5,'filled','b');

% write into a file
trnPts = fopen('to_pts.txt','w');
fprintf(trnPts,'%f %f %f\n',transpose(tpts));
fclose(trnPts);

%% before convergence

% choose approx S R and T 
rx = 0;
ry = 85;
rz = 5;
eul = [deg2rad(rx) deg2rad(ry) deg2rad(rz)];
rotmZYX = eul2rotm(eul);
sl = 1.1;
trl = [0; 0; 0];

smat = [sl 0 0 0; 0 sl 0 0 ; 0 0 sl 0; 0 0 0 1];
tmat = [rotmZYX trl];
tmat = [tmat ; 0 0 0 1];
tmat = smat * tmat;

% points before T convergence
gpts=[];
for idx=1:length(fpts)
    pt = [transpose(fpts(idx,:)); 1];
    tpt = tmat * pt;
    gpts = [gpts; transpose(tpt)];
end
gpts(:,4) = [];
hold on;
scatter3(gpts(:,1),gpts(:,2),gpts(:,3),5,'filled','o');

legend('from pts','to pts', 'before optimization');


sim3afile = fopen('sim3_approximate.txt','w');
for i=1:3
    for j=1:3
        fprintf(sim3afile,'%f ',rotmZYX(i,j));
    end
end

fprintf(sim3afile,'%f %f %f ',transpose(trl));
fprintf(sim3afile,'%f\n',sl);
fclose(sim3afile);

%% after convergence
%  SET A BREAK POINT HERE AND CONTINUE AFTER CERES EXECUTION
sim3efile = fopen('sim3_converged.txt','r');
format = '%f %f %f %f %f %f %f %f %f %f %f %f %f\n';
params = fscanf(sim3efile,format);
fclose(sim3efile);

rotmZYX = [transpose(params(1:3)); transpose(params(4:6)); transpose(params(7:9))];
sl = params(13);
trl = params(10:12);
smat = [sl 0 0 0; 0 sl 0 0 ; 0 0 sl 0; 0 0 0 1];
tmat = [rotmZYX trl];
tmat = [tmat ; 0 0 0 1];
tmat = smat * tmat;

% points before T convergence
cpts=[];
for idx=1:length(fpts)
    pt = [transpose(fpts(idx,:)); 1];
    tpt = tmat * pt;
    cpts = [cpts; transpose(tpt)];
end
cpts(:,4) = [];
figure;
scatter3(fpts(:,1),fpts(:,2),fpts(:,3),5,'filled','r');
hold on;
scatter3(tpts(:,1),tpts(:,2),tpts(:,3),5,'filled','b');
hold on;
scatter3(cpts(:,1),cpts(:,2),cpts(:,3),5,'filled','g');

legend('from pts','to pts', 'after optimization');

