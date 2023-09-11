close all;
clear;
clc;

%         theta |      d |         a |     alpha |     type|   offset |           
R(1) = Link([0       0.1035        0            pi/2    0], 'standard' );
R(2) = Link([0       0            0.35          0       0], 'standard' );
R(3) = Link([0       0            0.2253        0       0], 'standard' );
R(4) = Link([0       0            0.1702        pi/2    0], 'standard' );
R(5) = Link([0       0            0             pi/2    ], 'standard' );
R(6) = Link([0       0.0982       0             0       0], 'standard' );
R(1).qlim=[-150*pi/180,150*pi/180];
R(2).qlim=[-pi/2,pi/2];
R(3).qlim=[-pi/2,pi/2];
R(4).qlim=[-160*pi/180,160*pi/180];
R(5).qlim=[-pi/2,pi/2];
R(6).qlim=[-162*pi/180,162*pi/180];
Z1=SerialLink(R,'name','Z1','base',transl(0,0,-0.1035));  


hold on

axis([-1.5, 1.5, -1.5, 1.5, -1.5, 1.5])
Z1.plot([0 pi/4 150*pi/180 -15*pi/180 pi/2 0]);  
teach(Z1); 

%% 蒙特卡洛法求解工作空间点云
N=20000;   

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



theta8=(limitmin_R(1)+(limitmax_R(1)-limitmin_R(1))*rand(N,1))*pi/180; %关节左1限制
theta9=(limitmin_R(2)+(limitmax_R(2)-limitmin_R(2))*rand(N,1))*pi/180; %关节左1限制
theta10=(limitmin_R(3)+(limitmax_R(3)-limitmin_R(3))*rand(N,1))*pi/180; %关节左1限制
theta11=(limitmin_R(4)+(limitmax_R(4)-limitmin_R(4))*rand(N,1))*pi/180; %关节左1限制
theta12=(limitmin_R(5)+(limitmax_R(5)-limitmin_R(5))*rand(N,1))*pi/180; %关节左1限制
theta13=(limitmin_R(6)+(limitmax_R(6)-limitmin_R(6))*rand(N,1))*pi/180; %关节左1限制



tt=[theta8,theta9,theta10,theta11,theta12,theta13];

Mticx=double(Z1.fkine(tt));

x1=reshape(Mticx(1,4,:),N,1);
y1=reshape(Mticx(2,4,:),N,1);
z1=reshape(Mticx(3,4,:),N,1);
pc_b(:,1) =x1;
pc_b(:,2) =y1;
pc_b(:,3) =z1;
pc_bw = pointCloud(pc_b);
pcwrite(pc_bw,'Z1.pcd');
plot3(x1,y1,z1,'r.','MarkerSize',0.5);%画出落点
hold on;

f = convhulln(pc_b);
figure,
axis equal
patch('vertices',pc_b,'faces',f,'facecolor','[0.3350 0.4780 0.2840]','FaceAlpha',0.3)

f1_idex = f(:);
for i = 1:length(f1_idex)
    f_pc(i,:) = pc_b(f1_idex(i),:);
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
FV2.vertices = pc_b;
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