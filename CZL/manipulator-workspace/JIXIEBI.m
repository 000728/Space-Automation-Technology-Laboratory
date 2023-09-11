%机械臂可达空间迅速求解
% Robotic Toolbox 9.10

clc;
clear;
L1 = Link([0       0.1035        0            pi/2    0], 'standard' );
L2 = Link([0       0            0.35          0       0], 'standard' );
L3 = Link([0       0            0.2253        0       0], 'standard' );
L4 = Link([0       0            0.1702        pi/2    0], 'standard' );
L5 = Link([0       0            0            pi/2    0], 'standard' );
L6 = Link([0       0.0982       0            0       0], 'standard' );

L1.qlim=[-3.1415,3.1415];
L2.qlim=[-3.0718,0.0698];
L3.qlim=[-2.8973,2.8973];
L4.qlim=[-3.0718,0.0698];
L5.qlim=[-2.8973,2.8973];
L6.qlim=[-0.0175,3.7525];


Puma = SerialLink([L1 L2 L3 L4 L5 L6],'name','Puma');
axis([-1 1 -1 1 -0.5 1.5]);
teach(Puma); 
Puma.plot([-pi/2 -pi/2 0 0 0 0])
hold on;
N=100000;    %随机次数

    %关节角度限制
limitmax_L1 = 150.0;
limitmin_L1 = -150.0;
limitmax_L2 = 90.0;
limitmin_L2 = -90.0;
limitmax_L3 = 90.0;
limitmin_L3 = -90.0;
limitmax_L4 = 160.0;
limitmin_L4 = -160.0;
limitmax_L5 = 90.0;
limitmin_L5 = -90.0;
limitmax_L6 = 162.0;
limitmin_L6 = -162.0;

theta1=(limitmin_L1+(limitmax_L1-limitmin_L1)*rand(N,1))*pi/180; %关节1限制
theta2=(limitmin_L2+(limitmax_L2-limitmin_L2)*rand(N,1))*pi/180; %关节2限制
theta3=(limitmin_L3+(limitmax_L3-limitmin_L3)*rand(N,1))*pi/180; %关节3限制
theta4=(limitmin_L4+(limitmax_L4-limitmin_L4)*rand(N,1))*pi/180; %关节4限制
theta5=(limitmin_L5+(limitmax_L5-limitmin_L5)*rand(N,1))*pi/180; %关节5限制
theta6=(limitmin_L6+(limitmax_L6-limitmin_L6)*rand(N,1))*pi/180; %关节6限制

qq=[theta1,theta2,theta3,theta4,theta5,theta6];

Mricx=Puma.fkine(qq);

x=reshape(Mricx(1,4,:),N,1);
y=reshape(Mricx(2,4,:),N,1);
z=reshape(Mricx(3,4,:),N,1);
pcData=plot3(x,y,z,'b.','MarkerSize',0.5);%画出落点
hold on;



figure(2)
X = double(x); Y=double(y); Z= double(z);    %获取点云坐标
alp = 0.1;region = 0.33;hole = 0.7;
shp = alphaShape(X,Y,Z,alp);    %生产点云的包络数据
plot(shp)        %显示点云包络
v= volume(shp);
title(['v= ',num2str(v) ,'m3']) %计算体积并显示





