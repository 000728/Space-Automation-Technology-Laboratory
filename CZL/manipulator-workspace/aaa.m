clear;
clc;
ptCloud=pcread('xz.pcd');
pcshow(ptCloud.Location,'b');
% 点云x y z坐标
x = ptCloud.Location(:,1);
y = ptCloud.Location(:,2);
z = ptCloud.Location(:,3);

figure
X = double(x); Y=double(y); Z= double(z);    %获取点云坐标
alp = 0.4;region = 0.33;hole = 0.1;
shp = alphaShape(X,Y,Z,alp);    %生产点云的包络数据
plot(shp)        %显示点云包络
v= volume(shp);
title(['v= ',num2str(v) ,'m3']) %计算体积并显示