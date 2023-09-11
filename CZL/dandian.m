clc;clear;close all

%       theta    d        a        alpha     offset
L1(1) = Link('d', 103.5 ,'a', 0 , 'alpha', pi/2,'offset',0,'standard');
L1(2) = Link('d', 0   ,'a', 350 , 'alpha', 0,'offset',0,'standard');
L1(3) = Link('d', 0   ,'a', 225.3 , 'alpha', 0,'offset',0,'standard');
L1(4) = Link('d', 0   ,'a', 170.2 , 'alpha', pi/2,'offset',0,'standard');
L1(5) = Link('d', 0   ,'a', 0 , 'alpha', pi/2,'offset',0,'standard');
L1(6) = Link('d', 98.2 ,'a', 0 , 'alpha', 0,'offset',0,'standard');
robot=SerialLink(L1,'name','p1');

    q1_s=-180;     q1_end=180;
    q2_s=-90;     q2_end=90;
    q3_s=-90;     q3_end=90;
    q4_s=-160;     q4_end=160;
    q5_s=-90;     q5_end=90;
    q6_s=-162;     q6_end=162;
    %计算点数
     num=1;
 
 du=pi/180;  %度
    radian=180/pi; %弧度

 
%% 求取工作空间
    %设置轴关节随机分布,轴5不对工作范围产生影响，设置为0
    q1_rand = q1_s + rand(num,1)*(q1_end - q1_s);
    q2_rand = q2_s + rand(num,1)*(q2_end - q2_s);
    q3_rand = q3_s + rand(num,1)*(q3_end - q3_s);
    q4_rand = q4_s + rand(num,1)*(q4_end - q4_s);
    q5_rand = q5_s + rand(num,1)*(q5_end - q5_s);
    q6_rand = q6_s + rand(num,1)*(q6_end - q6_s);
    q = [q1_rand q2_rand q3_rand q4_rand q5_rand q6_rand ]*du;
    
    %正运动学计算工作空间
    tic;
    %臂1
    T1_cell = cell(num,1);
    for i=1:1:num
        [T1_cell{i}]= robot.fkine(q(i,:));%正向运动学仿真函数
    end
        disp(['运行时间：',num2str(toc)]);
 %% 分析结果
    t1=clock;
%     figure('name','机械臂工作空间')
    hold on
    
    plotopt = {'noraise', 'nowrist', 'nojaxes', 'delay',0};
    
    %绘制机械臂位置

    robot.plot([pi/2,0,pi/2,-pi/2,0,0], plotopt{:});
     figure1_x=256;
     figure1_y=297;
     figure1_z=460;
     
     plot3(figure1_x,figure1_y,figure1_z,'r.','MarkerSize',20)
     
%%
A=[figure1_x figure1_y figure1_z];%以A为圆心，画球的外围点



%% 
r=10;    
thetaaa=0:30*pi/180:2*pi;   
x=r*cos(thetaaa);  
y=r*sin(thetaaa);   
z=zeros(1,length(thetaaa));
hold on
plot3(x,y,z,'.');      
Y=[x;y;z];%最初外围点
k=1;
for t=0:pi/6:2*pi
T=[1 0 0;0 cos(t) -sin(t);0 sin(t) cos(t)];
NEW(:,:,k)=T*Y;
k=k+1;
end
hold on
plot3(NEW(1,:,1),NEW(2,:,1),NEW(3,:,1),'b.');              
hold on
plot3(NEW(1,:,2),NEW(2,:,2),NEW(3,:,2),'b.');              
hold on
plot3(NEW(1,:,3),NEW(2,:,3),NEW(3,:,3),'b.');              
hold on
plot3(NEW(1,:,4),NEW(2,:,4),NEW(3,:,4),'b.');              
hold on
plot3(NEW(1,:,5),NEW(2,:,5),NEW(3,:,5),'b.');              
hold on
plot3(NEW(1,:,6),NEW(2,:,6),NEW(3,:,6),'b.');              
hold on
plot3(NEW(1,:,7),NEW(2,:,7),NEW(3,:,7),'b.');              

for i=1:num
YX(:,:,i)=[A(i,1);A(i,2);A(i,3)];
end

% NEW1=NEW+[1;1;1]

for i=1:num
ZH(3*i-2:3*i,1:13,:)=NEW+YX(:,:,i);
end


for k=1:3:3*num
    
for i=1:7
plot3(ZH(k,:,i),ZH(k+1,:,i),ZH(k+2,:,i),'b.');              
hold on
axis([-1000 1000 -1000 1000 -1000 1000 ])
end 

end

j=1;
for k=1:3:3*num
for i=1:7
ZZ1=ZH(k,:,i)-YX(1,1,i);
ZZ2=ZH(k+1,:,i)-YX(2,1,i);
ZZ3=ZH(k+2,:,i)-YX(3,1,i);
d=sqrt(ZZ1.^2+ZZ2.^2+ZZ3.^2);
T=[1 0 ZZ1/d YX(1,1,i)
    0 1 ZZ2/d YX(2,1,i)
    0 0 ZZ3/d YX(3,1,i)
    0 0 0 1];
Q=robot.ikine(T,[1 1 1 0 0 1]);
B(j)=length(Q)
j=j+1;
end
end

C=sum(B>0);