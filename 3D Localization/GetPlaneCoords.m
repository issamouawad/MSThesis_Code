function [a,b,c] = GetPlaneCoords(P2,P3)
%GETPLANECOORDS Summary of this function goes here
%   Detailed explanation goes here
v = cross(P2(1:3),P3(1:3));
a = v(1);
b = v(2);
c = v(3);
end

