 
        
clc;
clear;
L1 = Link([0,0,0,0,0],'modified');
L2 = Link([0,0,0,-pi/2,0],'modified');
L3 = Link([0,0.15,0.4318,0,0],'modified');
L4 = Link([0,0.4318,0.02,-pi/2,0],'modified');
L5 = Link([0,0,0,pi/2,0],'modified');
L6 = Link([0,0,0,-pi/2,0],'modified');

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
N=50000;    %随机次数

    %关节角度限制
limitmax_1 = 180.0;
limitmin_1 = -180.0;
limitmax_2 = 4.0;
limitmin_2 = -176.0;
limitmax_3 = 166.0;
limitmin_3 = -166.0;
limitmax_4 = 4.0;
limitmin_4 = -176.0;
limitmax_5 = 166.0;
limitmin_5 = -166.0;
limitmax_6 = 215.0;
limitmin_6 = 1.0;

theta1=(limitmin_1+(limitmax_1-limitmin_1)*rand(N,1))*pi/180; %关节1限制
theta2=(limitmin_2+(limitmax_2-limitmin_2)*rand(N,1))*pi/180; %关节2限制
theta3=(limitmin_3+(limitmax_3-limitmin_3)*rand(N,1))*pi/180; %关节3限制
theta4=(limitmin_4+(limitmax_4-limitmin_4)*rand(N,1))*pi/180; %关节4限制
theta5=(limitmin_5+(limitmax_5-limitmin_5)*rand(N,1))*pi/180; %关节5限制
theta6=(limitmin_6+(limitmax_6-limitmin_6)*rand(N,1))*pi/180; %关节6限制

qq=[theta1,theta2,theta3,theta4,theta5,theta6];

Mricx=double(Puma.fkine(qq));

x=reshape(Mricx(1,4,:),[],1);
y=reshape(Mricx(2,4,:),[],1);
z=reshape(Mricx(3,4,:),[],1);
plot3(x,y,z,'b.','MarkerSize',0.5);%画出落点
hold on;
pc(:,1) =x;
pc(:,2) =y;
pc(:,3) =z;
pc = pointCloud(pc);
pcwrite(pc,'pc.pcd');
figure,pcshow(pc),title('单独的点云')