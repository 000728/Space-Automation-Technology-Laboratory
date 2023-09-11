
close all;
clear;
clc;

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


p560L=SerialLink(L,'name','LEFT');

view(3)
hold on
grid on
axis([-1.5, 1.5, -1.5, 1.5, -1.0, 2.5])
data=load('DD1.txt');
x=data(:,1);
y=data(:,2);
z=data(:,3);
plot3(x,y,z,'r.','MarkerSize',10)
p560L.plot([0 pi/4 -pi/4 0  0 0 0 ])
teach(p560L)
hold on;

%% 蒙特卡洛法求解工作空间点云
N=20000;   

%关节角度限制
limitmax_L(1) = 30.0;
limitmin_L(1) = -30.0;
limitmax_L(2) = 30.0;
limitmin_L(2) = -30.0;
limitmax_L(3) = 30.0;
limitmin_L(3) = -30.0;
limitmax_L(4) = 60.0;
limitmin_L(4) = -30.0;
limitmin_L(5) = -30.0;
limitmax_L(5) = 30.0;
limitmin_L(6) = -30.0;
limitmax_L(6) = 30.0;
limitmin_L(7) = -30.0;
limitmax_L(7) = 30.0;


theta1=(limitmin_L(1)+(limitmax_L(1)-limitmin_L(1))*rand(N,1))*pi/180; %关节左1限制
theta2=(limitmin_L(2)+(limitmax_L(2)-limitmin_L(2))*rand(N,1))*pi/180; %关节左2限制
theta3=(limitmin_L(3)+(limitmax_L(3)-limitmin_L(3))*rand(N,1))*pi/180; %关节左3限制
theta4=(limitmin_L(4)+(limitmax_L(4)-limitmin_L(4))*rand(N,1))*pi/180; %关节左4限制
theta5=(limitmin_L(5)+(limitmax_L(5)-limitmin_L(5))*rand(N,1))*pi/180; %关节左5限制
theta6=(limitmin_L(6)+(limitmax_L(6)-limitmin_L(6))*rand(N,1))*pi/180; %关节左6限制
theta7=(limitmin_L(7)+(limitmax_L(7)-limitmin_L(7))*rand(N,1))*pi/180; %关节左7限制

qq=[theta1,theta2,theta3,theta4,theta5,theta6,theta7];

Mricx=double(p560L.fkine(qq));

x=reshape(Mricx(1,4,:),N,1);
y=reshape(Mricx(2,4,:),N,1);
z=reshape(Mricx(3,4,:),N,1);
pc_r(:,1) =x;
pc_r(:,2) =y;
pc_r(:,3) =z;
pc_rw = pointCloud(pc_r);
pcwrite(pc_rw,'XMATE7PRO.pcd');
plot3(x,y,z,'b.','MarkerSize',0.5);%画出落点
hold on;


f = convhulln(pc_r);
figure,
hold on
grid on

axis([-1.5, 1.5, -1.5, 1.5, -1.0, 2.5])
patch('vertices',pc_r,'faces',f,'facecolor','[0.3350 0.4780 0.2840]','FaceAlpha',0.3)
f2_idex = f(:);
for i = 1:length(f2_idex)
    f_pc(i,:) = pc_r(f2_idex(i),:);
end
hold on 

n = 20;

f_pc2 = pointCloud(f_pc);
depth = 0.1;
gridstep = 0.001;
f_pc2 = pcdownsample(f_pc2,"gridAverage",gridstep);

mesh = pc2surfacemesh(f_pc2,'ball-pivot',0.5);
Vertices = double(mesh.Vertices);
Vertices1 = Vertices ./max(max(Vertices )).*n/2+7;
Vertices1(:,1) = Vertices1(:,1).*1.1+0.3;
Vertices1(:,2) = Vertices1(:,2).*1.2-2;
Vertices1(:,3) = Vertices1(:,3).*1.1;
figure,
axis equal
FV.vertices = mesh.Vertices;
FV.faces = mesh.Faces;
FV2.vertices = pc_r;
FV2.faces = f;

Volume=polygon2voxel(FV,[n n n],'auto');
for i = 1:size(Volume,3)
      Volume(:,:,i) = imfill(double(Volume(:,:,i)));
      Volume(i,:,:) = imfill(squeeze(double(Volume(i,:,:))));
      Volume(:,i,:) = imfill(squeeze(double(Volume(:,i,:))));
end
hold on,voxelPlot(double(Volume),'Color',[1,1,1]);axis tight;

%% 坐标索引
idx = find(Volume==1);
[pos(:,1),pos(:,2),pos(:,3)] = ind2sub(size(Volume), idx);
pos= pos-0.5;
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


function voxelPlot(mat, varargin)

if numDim(mat) ~= 3 || ~isfloat(mat)
    error('Input must be a 3D matrix in single or double precision.');
end

num_req_input_variables = 1;
transparency = 0.1;
axis_tight = false;
color_map = [1, 1, 0.4];    
if nargin < num_req_input_variables
    error('Incorrect number of inputs.');
elseif rem(nargin - num_req_input_variables, 2)
    error('Optional input parameters must be given as param, value pairs.');    
elseif ~isempty(varargin)
    for input_index = 1:2:length(varargin)
        switch varargin{input_index}
            case 'AxisTight'
                axis_tight = varargin{input_index + 1}; 
            case 'Color'
                color_map  = varargin{input_index + 1};                 
            case 'Transparency'
               transparency = varargin{input_index + 1};
            otherwise
                error('Unknown optional input.');
        end
    end
end


mat = mat ./ max(mat(:));

[IMAGE_3D_DATA] = image3Ddata(mat);  

voxel_num = (mat == 1);  
voxel_face_num = IMAGE_3D_DATA.voxel_patch_face_numbers(voxel_num, :);  
M_faces = IMAGE_3D_DATA.voxel_patch_faces(voxel_face_num, :);  
M_vertices = IMAGE_3D_DATA.corner_coordinates_columns_XYZ;  

hp2 = patch('Faces', M_faces, 'Vertices', M_vertices, 'EdgeColor', ...
    'black', 'CData', IMAGE_3D_DATA.voxel_patch_CData(voxel_face_num,:), ...
    'FaceColor', 'flat');  
set(hp2, 'FaceAlpha', transparency);
view(45, 30); 
axis equal;
box on;
colormap(color_map); 
grid on;  
axis equal;

xlabel('y [voxels]');
ylabel('x [voxels]');
zlabel('z [voxels]');

if ~axis_tight
    sz = size(mat);
    set(gca, 'XLim', [0.5, sz(2) + 0.5], ...
             'YLim', [0.5, sz(1) + 0.5], ...
             'ZLim', [0.5, sz(3) + 0.5]);
end

end