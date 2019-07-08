function [inside] = TestPointInPyramid(p,a1,b1,c1,a2,b2,c2,a3,b3,c3,a4,b4,c4)
%TESTPOINTINPYRAMID Summary of this function goes here
%   Detailed explanation goes here

left_plane_value  = a4 *p(1) + b4*p(2) + c4*p(3);
right_plane_value  = a2 *p(1) + b2*p(2) + c2*p(3);
if((left_plane_value>0 && right_plane_value<0) ||(left_plane_value<0 && right_plane_value>0))
    inside= false;
    return;
end
down_plane_value  = a3 *p(1) + b3*p(2) + c3*p(3);
if((down_plane_value>0 && right_plane_value<0) ||(down_plane_value<0 && right_plane_value>0))
    inside=false;
    return;
end
up_plane_value  = a1 *p(1) + b1*p(2) + c1*p(3);
if((up_plane_value>0 && right_plane_value<0) ||(up_plane_value<0 && right_plane_value>0))
    inside=false;
    return;
end
inside = true;
end

