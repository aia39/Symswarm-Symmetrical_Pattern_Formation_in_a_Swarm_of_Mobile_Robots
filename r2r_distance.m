function d = r2r_distance(n, r)
%THEORETICAL R2R DISTANCE
%Parameters
%n = number of robots, r = desired radius
c = 360/n;
d = 2*r*sind(c/2);
end