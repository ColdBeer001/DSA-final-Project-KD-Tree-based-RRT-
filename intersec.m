function result = intersec(obstacle,o_num,segment)
% check whether there exists collision between a  cuboid obstalce and segment 
% result: true for collision, false for free
% obstacle: structure array,cuboid 



% unfold cuboid obstacle to 8 vertex
vertex=zeros(3,8);
vertex(:,1)=obstacle.center(o_num,:)'+ ...
[-obstacle.width_x(o_num)/2;-obstacle.length_y(o_num)/2;-obstacle.height_z(o_num)/2];
vertex(:,2)=obstacle.center(o_num,:)'+ ...
[-obstacle.width_x(o_num)/2;obstacle.length_y(o_num)/2;-obstacle.height_z(o_num)/2];
vertex(:,3)=obstacle.center(o_num,:)'+ ...
[obstacle.width_x(o_num)/2;obstacle.length_y(o_num)/2;-obstacle.height_z(o_num)/2];
vertex(:,4)=obstacle.center(o_num,:)'+ ...
[obstacle.width_x(o_num)/2;-obstacle.length_y(o_num)/2;-obstacle.height_z(o_num)/2];
vertex(:,5)=obstacle.center(o_num,:)'+ ...
[-obstacle.width_x(o_num)/2;-obstacle.length_y(o_num)/2;obstacle.height_z(o_num)/2];
vertex(:,6)=obstacle.center(o_num,:)'+ ...
[-obstacle.width_x(o_num)/2;obstacle.length_y(o_num)/2;obstacle.height_z(o_num)/2];
vertex(:,7)=obstacle.center(o_num,:)'+ ...
[obstacle.width_x(o_num)/2;obstacle.length_y(o_num)/2;obstacle.height_z(o_num)/2];
vertex(:,8)=obstacle.center(o_num,:)'+ ...
[obstacle.width_x(o_num)/2;-obstacle.length_y(o_num)/2;obstacle.height_z(o_num)/2];

% using naive intersection-check the projection onto the x,y,z-panel respectively 
% for collision check
xy_int=false;
yz_int=false;
zx_int=false;
x_int=false;
y_int=false;
z_int=false;
xy_proj=segment(1:2,:);
yz_proj=segment(2:3,:);
xz_proj=[segment(1,:);segment(3,:)];
xz_verp=[vertex(1,:);vertex(3,:)];

% check whether path's projection lies inside obstacle in any panel
segx_min=min(segment(1,:));segy_min=min(segment(2,:));segz_min=min(segment(3,:));
segx_max=max(segment(1,:));segy_max=max(segment(2,:));segz_max=max(segment(3,:));
obsx_min=min(vertex(1,:));obsy_min=min(vertex(2,:));obsz_min=min(vertex(3,:));
obsx_max=max(vertex(1,:));obsy_max=max(vertex(2,:));obsz_max=max(vertex(3,:));

if((segx_min>=obsx_min&&segx_max<=obsx_max)||(obsx_min>=segx_min&&obsx_max<=segx_max))
   x_int=true; 
end
if((segy_min>=obsy_min&&segy_max<=obsy_max)||(obsy_min>=segy_min&&obsy_max<=segy_max))
   y_int=true;
end
if((segz_min>=obsz_min&&segz_max<=obsz_max)||(obsz_min>=segz_min&&obsz_max<=segz_max))
   z_int=true;
end
xy_int=x_int&&y_int; yz_int=y_int&&z_int; zx_int=z_int&&x_int;
% check xy-panel, z fixed
    if(intersec_2D(xy_proj,vertex(1:2,1:2))==true || ...
        intersec_2D(xy_proj,vertex(1:2,2:3))==true || ...
        intersec_2D(xy_proj,vertex(1:2,3:4))==true || ...
        intersec_2D(xy_proj,[vertex(1:2,1),vertex(1:2,4)])==true )
        xy_int=true;
    end
% yz-panel, x fixed
    if(xy_int==false)
        result=false; % bypass proceeding check 
        return
    elseif( intersec_2D(yz_proj,vertex(2:3,1:2))==true || ...
        intersec_2D(yz_proj,[vertex(2:3,2),vertex(2:3,6)])==true || ...
        intersec_2D(yz_proj,vertex(2:3,5:6))==true || ...
        intersec_2D(yz_proj,[vertex(2:3,5),vertex(2:3,1)])==true )
        yz_int=true;
    end
% zx-panel, y fixed
    if(yz_int==false)
        result=false;
        return
    elseif( intersec_2D(xz_proj,[xz_verp(:,1),xz_verp(:,4)])==true || ...
        intersec_2D(xz_proj,[xz_verp(:,4),xz_verp(:,8)])==true || ...
        intersec_2D(xz_proj,[xz_verp(:,8),xz_verp(:,5)])==true || ...
        intersec_2D(xz_proj,[xz_verp(:,5),xz_verp(:,1)])==true )
        zx_int=true;
    end
    
    if(zx_int==true&&yz_int==true&&xy_int==true)
        result=true;
    else
        result=false;
    end
    return
end
function result=intersec_2D(Line1,Line2)
    % check whether two segment intersect, Each column represents one point
    % return true for intersected, false for free
    result=false;
    min_x1=min(Line1(1,1),Line1(1,2));
    max_x1=max(Line1(1,1),Line1(1,2));
    
    min_x2=min(Line2(1,1),Line2(1,2));
    max_x2=max(Line2(1,1),Line2(1,2));
    % check whether one line lines beneath 
    
    % calculate the intersection of the two lines, y=kx+b
    
    k1=(Line1(2,1)-Line1(2,2))/(Line1(1,1)-Line1(1,2));
    b1=Line1(2,2)-k1*Line1(1,2);
    
    k2=(Line2(2,1)-Line2(2,2))/(Line2(1,1)-Line2(1,2));
    b2=Line2(2,2)-k2*Line2(1,2);
    
    
    if(k1==Inf&&k2==Inf)
        result=false;
        return
    elseif (k1==Inf||k1==-Inf)
        in_x=Line1(1,1);
    elseif(k2==Inf||k2==-Inf)
        in_x=Line2(1,1);
    else
        in_x=(b2-b1)/(k1-k2);
    end
    if(in_x>=min_x1&&in_x<=max_x1&&in_x>=min_x2&&in_x<=max_x2)
       result=true; 
       return
    end
end