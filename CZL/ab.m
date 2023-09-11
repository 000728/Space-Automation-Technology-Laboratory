clc;clear
L1(1) = Link('d', 103.5 ,'a', 0 , 'alpha', pi/2,'offset',0,'standard');
L1(2) = Link('d', 0   ,'a', 350 , 'alpha', 0,'offset',0,'standard');
L1(3) = Link('d', 0   ,'a', 225.3 , 'alpha', 0,'offset',0,'standard');
L1(4) = Link('d', 0   ,'a', 170.2 , 'alpha', pi/2,'offset',0,'standard');
L1(5) = Link('d', 0   ,'a', 0 , 'alpha', pi/2,'offset',0,'standard');
L1(6) = Link('d', 98.2 ,'a', 0 , 'alpha', 0,'offset',0,'standard');
robot=SerialLink(L1,'name','p1','base',transl(0,0,-103.5));
robot.plot([pi/2,0,pi/2,-pi/2,0,0]);
teach(robot); 
hold on
 num=1;
   figure1_x=190;
    figure1_y=114;
    figure1_z=38;
plot3(figure1_x,figure1_y,figure1_z,'r.','MarkerSize',10)
     
%% 
A=[figure1_x figure1_y figure1_z];

%% 
r=38;    
thetaaa=0:30*pi/180:2*pi;   
x=r*cos(thetaaa);  
y=r*sin(thetaaa);   
z=zeros(1,length(thetaaa));
hold on
plot3(x,y,z,'.');      
Y=[x;y;z];
k=1;
for t=0:pi/6:pi
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
axis([-1000 1000 -1000 1000 -1000 1200 ])
end 

end

H=1;
for k=1:3:3*num
for i=1:6
for j=2:12
ZZ1=ZH(k,j,i)-YX(1,1);
ZZ2=ZH(k+1,j,i)-YX(2,1);
ZZ3=ZH(k+2,j,i)-YX(3,1);
d=sqrt(ZZ1.^2+ZZ2.^2+ZZ3.^2);
ZZ=[ZZ1;ZZ2;ZZ3];
s=rng(1);
XX = rand(3,1);
XX = XX - dot(XX,ZZ)*ZZ/norm(ZZ);
XX = XX / norm(XX)
YY = cross(ZZ,XX);
YY = YY / norm(YY)
ZZ = ZZ/d
% Î»×Ë¾ØÕó
R = [XX, YY, ZZ]; % Ðý×ª¾ØÕó
p = [YX(1,1);YX(2,1);YX(3,1)]; 
T = [R, p; 0 0 0 1];
Q=robot.ikine(T);
B(H)=length(Q);
H=H+1;
end
end
end
C=sum(B>0)


