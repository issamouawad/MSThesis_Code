function [BB_3D,centroid] = Get_Lidar_BB(BB_2D, InvM,pointCloud,order)
if ~exist('order','var')
     % third parameter does not exist, so default it to something
      order=1;
 end
n=5;
[BB_2D(1);BB_2D(2);1]
pos3d1=InvM*[BB_2D(1);BB_2D(2);1];

pos3d2=InvM*[BB_2D(3);BB_2D(2);1];

pos3d3=InvM*[BB_2D(3);BB_2D(4);1];


pos3d4=InvM*[BB_2D(1);BB_2D(4);1];
% figure()
% pcshow(pointCloud)
% hold on
% plot3([0 pos3d1(1)*n],[0 min(100,pos3d1(2)*n)] , [0 min(100,pos3d1(3)*n)] )
% hold on 
% plot3([0 pos3d2(1)*n],[0 min(100,pos3d2(2)*n)] , [0 min(100,pos3d2(3)*n)] )
% hold on 
% plot3([0 pos3d3(1)*n],[0 min(100,pos3d3(2)*n)] , [0 min(100,pos3d3(3)*n)] )
% hold on 
% plot3([0 pos3d4(1)*n],[0 min(100,pos3d4(2)*n)] , [0 min(100,pos3d4(3)*n)] )
% hold off
[a1,b1,c1] = GetPlaneCoords(pos3d1,pos3d2);
[a2,b2,c2] = GetPlaneCoords(pos3d2,pos3d3);
[a3,b3,c3] = GetPlaneCoords(pos3d3,pos3d4);
[a4,b4,c4] = GetPlaneCoords(pos3d4,pos3d1);

points3d = [];
dist_threshold = 0.75;
for index1=1:size(pointCloud.Location)
    
        
    x=pointCloud.Location(index1,1);
    y=pointCloud.Location(index1,2);
    z=pointCloud.Location(index1,3);
   
   if(TestPointInPyramid([x,y,z],a1,b1,c1,a2,b2,c2,a3,b3,c3,a4,b4,c4))
       
        
        points3d = [points3d; x,y,z,norm([x,y,z],2)];
        %points3d = [points3d; x,y,z,y];
   end
   end
   cutting = 0;
   
   if(size(points3d,1)<=0)
       BB_3D = [0 0;0 0;0 0; 0 0];
   
   else
   B = sortrows(points3d,4);
prev = B(1,4);
start =1;

cur_order = 1;
for j=2:size(B,1)

    if(B(j,4)-prev>dist_threshold )
        
        if(cur_order==order && j>2)
            
        break
        end
        cur_order = cur_order+1;
        start = j;
    end
    prev = B(j,4);
end
cutted_points = B(start:j-1,:);
centroid = [sum(cutted_points(:,1))/size(cutted_points,1)
    sum(cutted_points(:,2))/size(cutted_points,1)
    sum(cutted_points(:,3))/size(cutted_points,1)];
obb_points = cutted_points(:,1:2);
covpoints = cov(obb_points);
[V,D] = eig(covpoints);
tv = transpose(V);
rotated_points = obb_points*inv(tv);
mina = min(rotated_points)-1.5;
maxa = max(rotated_points)+1.5;
diff = (maxa - mina)*0.5;
center = mina + diff;
corners = [center+[-diff(1),-diff(2)];center+[diff(1),-diff(2)];center+[diff(1),diff(2)];center+[-diff(1),diff(2)]];
BB_3D = corners*tv;
   end
end
