% %工作空间计算
clc;
clear;
close all
format short;%保留精度
A1=load('dd1.txt');
x1=A1(:,1) ; y1=A1(:,2);z1=A1(:,3);
xmin=0 ; 
xmax=1 ; 
ymin=0 ; 
ymax=1 ; 
zmin=0 ; 
zmax=1;
 [m,~]=size(x1);%数据点个数

%% solution
L1(1) = Link('d', 103.5 ,'a', 0 , 'alpha', pi/2,'offset',0,'standard');
L1(2) = Link('d', 0   ,'a', 350 , 'alpha', 0,'offset',-pi/2,'standard');
L1(3) = Link('d', 0   ,'a', 225.3 , 'alpha', 0,'offset',0,'standard');
L1(4) = Link('d', 0   ,'a', 170.2 , 'alpha', pi/2,'offset',0,'standard');
L1(5) = Link('d', 0   ,'a', 0 , 'alpha', pi/2,'offset',0,'standard');
L1(6) = Link('d', 98.2 ,'a', 0 , 'alpha', 0,'offset',0,'standard');
robot=SerialLink(L1,'name','p1');

    figure1_x=A1(:,1);
    figure1_y=A1(:,2);
    figure1_z=A1(:,3);

A=[figure1_x figure1_y figure1_z];%以A为圆心，画球的外围点


r=10;    
thetaaa=0:30*pi/180:2*pi;   
x=r*cos(thetaaa);  
y=r*sin(thetaaa);   
z=zeros(1,length(thetaaa));
hold on
plot3(x,y,z,'.');      
Y=[x;y;z];%最初外围点
k=1;
for t=0:pi/6:pi
T=[1 0 0;0 cos(t) -sin(t);0 sin(t) cos(t)];
NEW(:,:,k)=T*Y;
k=k+1;
end
num=length(A(:,1))
for i=1:num
YX(:,:,i)=[A(i,1);A(i,2);A(i,3)];
end

% NEW1=NEW+[1;1;1]

for i=1:num
ZH(3*i-2:3*i,1:13,:)=NEW+YX(:,:,i);%ZH为偏移圆心的球点,每三行是一个球
end


for k=1:3:3*num
    
for i=1:7
plot3(ZH(k,:,i),ZH(k+1,:,i),ZH(k+2,:,i),'b.');              
hold on
axis([-1000 1000 -1000 1000 -1000 1000 ])
end 

end
% ZH的第一行是x，第二行是y，第三行是z
H=1;
for k=1:3:3*num
for i=1:7
for j=1:13
ZZ1=ZH(k,j,i)-YX(1,1);
ZZ2=ZH(k+1,j,i)-YX(2,1);
ZZ3=ZH(k+2,j,i)-YX(3,1);
d=sqrt(ZZ1.^2+ZZ2.^2+ZZ3.^2);
ZZ=[ZZ1;ZZ2;ZZ3];
XX = randn(3,1);
XX = XX - dot(XX,ZZ)*ZZ/norm(ZZ); % 使x与z垂直
XX = XX / norm(XX);
YY = cross(ZZ,XX);% 计算Y轴方向向量，使得X、Y、Z构成一个右手系
% 计算位姿矩阵
R = [XX, YY, ZZ]; % 旋转矩阵
p = [YX(1,1);YX(2,1);YX(3,1)]; % 机器人在世界坐标系下的位置向量
T = [R, p; 0 0 0 1]; % 位姿矩阵
Q=robot.ikine(T,[1 1 1 0 0 1]);
B(H)=length(Q);
H=H+1;
end
end
end
C=sum(B>0)
 data(i, 4) = B(i);


dlmwrite('dd1.txt', data, 'delimiter', '\t', 'precision', '%.6f');


