close all;
clear;
clc;
%% 建立双臂机器人模型
%   theta |         d |         a |     alpha |     type|   offset |
L(1)=Link([0       0.404       0       -pi/2            0   ]); 
L(2)=Link([0       0           0        pi/2            0   ]);
L(3)=Link([0       0.4375     0       -pi/2             0   ]);
L(4)=Link([0       0           0        pi/2            0   ]);
L(5)=Link([0       0.4125      0       -pi/2            0   ]);
L(6)=Link([0       0           0        pi/2            0   ]);
L(7)=Link([0       0.2755      0         0              0   ]);




L(1).qlim=[-2.96,2.96];
L(2).qlim=[-2.09,2.09];
L(3).qlim=[-2.96,2.96];
L(4).qlim=[-2.09,2.09];
L(5).qlim=[-2.96,2.96];
L(6).qlim=[-2.09,2.09];
L(7).qlim=[-3.14,3.14];


p560L=SerialLink(L,'name','LEFT','base',transl(-0.5,0,0));

           
R(1) = Link([0       0.1035        0            pi/2    0], 'standard' );
R(2) = Link([0       0            0.35          0       0], 'standard' );
R(3) = Link([0       0            0.2253        0       0], 'standard' );
R(4) = Link([0       0            0.1702        pi/2    0], 'standard' );
R(5) = Link([0       0            0             pi/2    0], 'standard' );
R(6) = Link([0       0.0982       0             0       0], 'standard' );


R(1).qlim=[-3.1415,3.1415];
R(2).qlim=[-3.0718,0.0698];
R(3).qlim=[-2.8973,2.8973];
R(4).qlim=[-3.0718,0.0698];
R(5).qlim=[-2.8973,2.8973];
R(6).qlim=[-0.0175,3.7525];

p560R=SerialLink(R,'name','RIGHT','base',transl(0.5,0,0));

view(3)
hold on
grid on
axis([-1.5, 1.5, -1.5, 1.5, -1.0, 2.0])

p560R.plot([0 pi/4 pi/4 0 0 0 ]) 
p560L.plot([0 pi/4 -pi/4 0  0 0 0 ])


teach(p560L);
teach(p560R);

hold on

%% 蒙特卡洛法求解工作空间点云
N=10000;   

%关节角度限制
limitmax_L(1) = 170.0;
limitmin_L(1) = -170.0;
limitmax_L(2) = 120.0;
limitmin_L(2) = -120.0;
limitmax_L(3) = 170.0;
limitmin_L(3) = -170.0;
limitmax_L(4) = 120.0;
limitmin_L(4) = -120.0;
limitmin_L(5) = -170.0;
limitmax_L(5) = 170.0;
limitmin_L(6) = -120.0;
limitmax_L(6) = 120.0;
limitmin_L(7) = -360.0;
limitmax_L(7) = 360.0;
limitmax_R(1) = 180.0;
limitmin_R(1) = -180.0;
limitmax_R(2) = 90.0;
limitmin_R(2) = -90.0;
limitmax_R(3) = 90.0;
limitmin_R(3) = -90.0;
limitmax_R(4) = 160.0;
limitmin_R(4) = -160.0;
limitmax_R(5) = 90.0;
limitmin_R(5) = -90.0;
limitmax_R(6) = 162.0;
limitmin_R(6) = -162.0;





theta1=(limitmin_L(1)+(limitmax_L(1)-limitmin_L(1))*rand(N,1))*pi/180; %关节左1限制
theta2=(limitmin_L(2)+(limitmax_L(2)-limitmin_L(2))*rand(N,1))*pi/180; %关节左2限制
theta3=(limitmin_L(3)+(limitmax_L(3)-limitmin_L(3))*rand(N,1))*pi/180; %关节左3限制
theta4=(limitmin_L(4)+(limitmax_L(4)-limitmin_L(4))*rand(N,1))*pi/180; %关节左4限制
theta5=(limitmin_L(5)+(limitmax_L(5)-limitmin_L(5))*rand(N,1))*pi/180; %关节左4限制
theta6=(limitmin_L(6)+(limitmax_L(6)-limitmin_L(6))*rand(N,1))*pi/180; %关节左4限制
theta7=(limitmin_L(7)+(limitmax_L(7)-limitmin_L(7))*rand(N,1))*pi/180; %关节左4限制
theta8=(limitmin_R(1)+(limitmax_R(1)-limitmin_R(1))*rand(N,1))*pi/180; %关节左1限制
theta9=(limitmin_R(2)+(limitmax_R(2)-limitmin_R(2))*rand(N,1))*pi/180; %关节左1限制
theta10=(limitmin_R(3)+(limitmax_R(3)-limitmin_R(3))*rand(N,1))*pi/180; %关节左1限制
theta11=(limitmin_R(4)+(limitmax_R(4)-limitmin_R(4))*rand(N,1))*pi/180; %关节左1限制
theta12=(limitmin_R(5)+(limitmax_R(5)-limitmin_R(5))*rand(N,1))*pi/180; %关节左1限制
theta13=(limitmin_R(6)+(limitmax_R(6)-limitmin_R(6))*rand(N,1))*pi/180; %关节左1限制
%% 左臂单独求解
qq=[theta1,theta2,theta3,theta4,theta5,theta6,theta7];

Mricx=double(p560L.fkine(qq));

x=reshape(Mricx(1,4,:),N,1);
y=reshape(Mricx(2,4,:),N,1);
z=reshape(Mricx(3,4,:),N,1);


%% 右臂单独求解
tt=[theta8,theta9,theta10,theta11,theta12,theta13];

Mticx=double(p560R.fkine(tt));

x1=reshape(Mticx(1,4,:),N,1);
y1=reshape(Mticx(2,4,:),N,1);
z1=reshape(Mticx(3,4,:),N,1);


pc(:,1) =x;
pc(:,2) =y;
pc(:,3) =z;
pc = pointCloud(pc);
pcwrite(pc,'zz.pcd');
figure(2),pcshow(pc),title('左臂单独的点云')
hold on;

pc1(:,1) =x1;
pc1(:,2) =y1;
pc1(:,3) =z1;
pc1 = pointCloud(pc1);
pcwrite(pc1,'x7.pcd');
figure(3),pcshow(pc1),title('右臂单独的点云')
hold on;

ptCloudA = pcread('zz.pcd');
ptCloudB = pcread('x7.pcd');
ptCloudOut = pcmerge(ptCloudA,ptCloudB,0.001);
pcwrite(ptCloudOut,'xz.pcd');
figure(4)
pcshow(ptCloudOut);
title('合并后的点云')