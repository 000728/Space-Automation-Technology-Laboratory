close all;
clear;
clc;
%% 建立双臂机器人模型
%   theta |         d |         a |     alpha |     type|   offset |
L(1)=Link([0       -0.072      0.150        0          0     pi/2  ],'modified'); 
L(2)=Link([0       0           0.022        pi/2       0    -pi/2  ],'modified');
L(3)=Link([0       0           0.285        0          0     -pi/2 ],'modified');
L(4)=Link([0       0.22        0.0035       -pi/2      0       0   ],'modified');


L(1).qlim=[-3.0,3.0];
L(2).qlim=[-3.0718,0.0698];
L(3).qlim=[-2.8973,2.8973];
L(4).qlim=[-3.0718,0.0698];

p560L=SerialLink(L,'name','LEFT');

           
R(1)=Link([0       -0.072      -0.15        0          0     pi/2  ],'modified'); 
R(2)=Link([0       0           0.022        pi/2       0    -pi/2  ],'modified');
R(3)=Link([0       0           0.285        0          0     -pi/2 ],'modified');
R(4)=Link([0       0.22        0.0035       -pi/2      0       0   ],'modified');


R(1).qlim=[-3.0,3.0];
R(2).qlim=[-3.0718,0.0698];
R(3).qlim=[-2.8973,2.8973];
R(4).qlim=[-3.0718,0.0698];

p560R=SerialLink(R,'name','RIGHT');

 
platform=SerialLink([0 0 0 0],'name','platform','modified');%虚拟腰部关节
platform.base=[1 0 0 0;
               0 1 0 0;
               0 0 1 0 ;
               0 0 0 1;]; %基座高度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   R
 
pR=SerialLink([platform,p560R],'name','R'); % 单独右臂模型

 
 
view(3)
hold on
grid on
axis([-1.5, 1.5, -1.5, 1.5, -1.0, 1.5])

pR.plot([0 pi/4 pi/4 0 0 ]) 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   L
pL=SerialLink([platform,p560L],'name','L'); % 单独左臂模型



teach(pL);
teach(pR);

pL.plot([0 -pi/4 pi/4 0  0 ])
hold on


%% 蒙特卡洛法求解工作空间点云
N=3000;   

%关节角度限制
limitmax_L(1) = 180.0;
limitmin_L(1) = -180.0;
limitmax_L(2) = 4.0;
limitmin_L(2) = -176.0;
limitmax_L(3) = 166.0;
limitmin_L(3) = -166.0;
limitmax_L(4) = 4.0;
limitmin_L(4) = -176.0;
limitmax_R(1) = 180.0;
limitmin_R(1) = -180.0;
limitmax_R(2) = 4.0;
limitmin_R(2) = -176.0;
limitmax_R(3) = 166.0;
limitmin_R(3) = -166.0;
limitmax_R(4) = 4.0;
limitmin_R(4) = -176.0;

theta1=(limitmin_L(1)+(limitmax_L(1)-limitmin_L(1))*rand(N,1))*pi/180; %关节左1限制
theta2=(limitmin_L(2)+(limitmax_L(2)-limitmin_L(2))*rand(N,1))*pi/180; %关节左2限制
theta3=(limitmin_L(3)+(limitmax_L(3)-limitmin_L(3))*rand(N,1))*pi/180; %关节左3限制
theta4=(limitmin_L(4)+(limitmax_L(4)-limitmin_L(4))*rand(N,1))*pi/180; %关节左4限制
theta5=(limitmin_R(1)+(limitmax_R(1)-limitmin_R(1))*rand(N,1))*pi/180; %关节右1限制
theta6=(limitmin_R(2)+(limitmax_R(2)-limitmin_R(2))*rand(N,1))*pi/180; %关节右2限制
theta7=(limitmin_R(3)+(limitmax_R(3)-limitmin_R(3))*rand(N,1))*pi/180; %关节右3限制
theta8=(limitmin_R(4)+(limitmax_R(4)-limitmin_R(4))*rand(N,1))*pi/180; %关节右4限制


%% 左臂单独求解
qq=[theta1,theta2,theta3,theta4];

Mricx=double(p560L.fkine(qq));

x=reshape(Mricx(1,4,:),N,1);
y=reshape(Mricx(2,4,:),N,1);
z=reshape(Mricx(3,4,:),N,1);
pc_r(:,1) =x;
pc_r(:,2) =y;
pc_r(:,3) =z;
pc_rw = pointCloud(pc_r);
pcwrite(pc_rw,'red.pcd');
plot3(x,y,z,'b.','MarkerSize',0.5);%画出落点
hold on;

%% 右臂单独求解
tt=[theta5,theta6,theta7,theta8];

Mticx=double(p560R.fkine(tt));

x1=reshape(Mticx(1,4,:),N,1);
y1=reshape(Mticx(2,4,:),N,1);
z1=reshape(Mticx(3,4,:),N,1);
pc_b(:,1) =x1;
pc_b(:,2) =y1;
pc_b(:,3) =z1;
pc_bw = pointCloud(pc_b);
pcwrite(pc_bw,'blue.pcd');
plot3(x1,y1,z1,'r.','MarkerSize',0.5);%画出落点
hold on;
%% 整体包络
f = convhulln(pc_r);
figure,
patch('vertices',pc_r,'faces',f,'facecolor','[0.3350 0.4780 0.2840]','FaceAlpha',0.3)

hold on 
f2 = convhulln(pc_b);
patch('vertices',pc_b,'faces',f2,'facecolor','[0.3350 0.4780 0.2840]','FaceAlpha',0.3)



%% 计算重叠部分
fv.faces = f;
fv.vertices = pc_r;
IN = in_polyhedron(fv,pc_b);
pc_b_in = pc_b(:,1:3).*IN;
pc_b_in(all(pc_b_in==0,2),:) = [];


fv2.faces = f2;
fv2.vertices = pc_b;
IN2 = in_polyhedron(fv2,pc_r);
pc_r_in = pc_r(:,1:3).*IN2;
pc_r_in(all(pc_r_in==0,2),:) = [];

pc_in = [pc_b_in;pc_r_in];



%% 重叠部分标注
[f3,vol] = convhulln(pc_in);
hold on ;
patch('vertices',pc_in,'faces',f3,'facecolor','red','FaceAlpha',1)
view(3),axis equal;lighting phong;
v = vol;
title(['v= ',num2str(v) ,'m3'])
%%
function inside = in_polyhedron(varargin)
if  isa(varargin{1}, 'triangulation') 
  [faces, vertices] = freeBoundary(varargin{1});
  points = varargin{2};
elseif isstruct(varargin{1})   
  ok = isfield(varargin{1}, 'vertices') && isfield(varargin{1}, 'faces');
  assert(ok, 'Structure FV must have "faces" and "vertices" fields' );
  faces    = varargin{1}.faces;
  vertices = varargin{1}.vertices;
  points   = varargin{2};
else                      
  faces    = varargin{1};
  vertices = varargin{2};
  points   = varargin{3};
end
clear varargin
if (size(points  ,1)==3 && size(points  ,2)~=3), points   = points';   end
if (size(vertices,1)==3 && size(vertices,2)~=3), vertices = vertices'; end
if (size(faces   ,1)==3 && size(faces   ,2)~=3), faces    = faces';    end
assert(size(points  ,2)==3, '"Points" array must be in Nx3 format');
assert(size(vertices,2)==3, '"Vertices" array must be in Nx3 format');
assert(size(faces   ,2)==3, '"Faces" array must be in Nx3 format');
vert0  = vertices(faces(:,1),:);
edge1  = vertices(faces(:,2),:)-vert0; 
edge2  = vertices(faces(:,3),:)-vert0;
N      = size(vert0,1);
clear vertices faces                   
eps    = 1e-10; 
inside = nan+zeros(size(points,1),1); 
while any(isnan(inside))
  dir  = repmat(rand(1,3)-0.5,N,1);   
  pvec = cross_prod(dir, edge2);
  det  = sum(edge1.*pvec,2);
  angleOK = (abs(det)>eps);  
  for iPoint = 1:size(points,1)
    if ~isnan(inside(iPoint))
      continue
    end
    tvec = -bsxfun(@minus, vert0, points(iPoint,:));
    u    = sum(tvec.*pvec,2)./det;    
    ok = (angleOK & u>-eps & u<=1.0+eps); 
    u = u(ok,:); 
    if ~any(ok)
      if ~any(u>eps & u<=1.0-eps) 
        inside (iPoint) = false;
      end
      continue
    end
    qvec = cross_prod(tvec(ok,:), edge1(ok,:)); 
    v = sum(dir  (ok,:).*qvec,2)./det(ok,:); 
    t = sum(edge2(ok,:).*qvec,2)./det(ok,:); 
    bary = [u, v, 1-u-v, t];      
    intersect = all(bary>-eps,2); 
    baryi = bary(intersect,:);    
    if all( min(abs(baryi),[], 2)>eps ) 
      nIntersect = sum(intersect);    
      inside(iPoint) = mod(nIntersect,2)>0; 
    end
    if any(max(bary,[], 2)<1+eps & abs(t)<eps) 
      inside(iPoint) = true;
    end
  end
end
inside = (inside~=0); 
function c=cross_prod(a,b)
c = [a(:,2).*b(:,3)-a(:,3).*b(:,2), a(:,3).*b(:,1)-a(:,1).*b(:,3), a(:,1).*b(:,2)-a(:,2).*b(:,1)];
end
end