%初始化
clear;
clc;
%% 建立机器人模型 
% a--连杆长度；alpha--连杆扭角；d--连杆偏距；theta--关节转角
%       theta    d           a        alpha     offset
SL1=Link([0     0            0          0         0     ],'modified'); 
SL2=Link([0     0            0        -pi/2       0     ],'modified');
SL3=Link([0     0.15         0.4318      0        0     ],'modified');
SL4=Link([0     0.4318       0.02     -pi/2       0     ],'modified');
SL5=Link([0     0            0         pi/2       0     ],'modified');
SL6=Link([0     0            0        -pi/2       0     ],'modified');

%定义关节范围
SL1.qlim=[-3.1415,3.1415];
SL2.qlim=[-3.0718,0.0698];
SL3.qlim=[-2.8973,2.8973];
SL4.qlim=[-3.0718,0.0698];
SL5.qlim=[-2.8973,2.8973];
SL6.qlim=[-0.0175,3.7525];

 % 连接连杆，机器人取名Puma
Puma=SerialLink([SL1 SL2 SL3 SL4 SL5 SL6],'name','Puma');
Puma.plot([-pi/2 -pi/2 0 0 0 0]);
axis([-1 1 -1 1 -0.5 1.5]);
xlabel('X/m')
ylabel('Y/m')
zlabel('Z/m')
teach(Puma);%可以自由改变角度
hold on;
Puma.display();
%% 蒙特卡洛法求工作空间 
%随机点数
n=5000;
%每个关节随机生成角度
q1= -3.1415+(3.1415-(-3.1415))*rand(n,1);
q2= -3.0718+(0.0698-(-3.0718))*rand(n,1);
q3= -2.8973+(2.8973-(-2.8973))*rand(n,1);
q4= -3.0718+(0.0698-(-3.0718))*rand(n,1);
q5= -2.8973+(2.8973-(-2.8973))*rand(n,1);
q6= -0.0175+(3.7525-(-0.0175))*rand(n,1);

for i=1:1:n
    %求末端位姿矩阵
    mod06=Puma.fkine([q1(i),q2(i),q3(i),q4(i),q5(i),q6(i)]);
    %动画显示
    Puma.plot([q1(i),q2(i),q3(i),q4(i),q5(i),q6(i)]);
    %绘制落点
    plot3(mod06(1,4),mod06(2,4),mod06(3,4),'b.');
    if mod06(1,4)>=0.&&mod06(1,4)<=1&&mod06(2,4)>=-0.5&&mod06(2,4)<=0.7&&mod06(3,4)>=0&&mod06(3,4)<=0.8
        color_Tag=0;
    else
        color_Tag=1;
    end
    point(i,:)={mod06(1,4),mod06(2,4),mod06(3,4),color_Tag};  %保存到point中
    hold on;
end   
 xlswrite('point.xls',point,'sheet1','A1'); %写入excel
 
