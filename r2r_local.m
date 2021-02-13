function r2r_tot = r2r_local(x,y)
%R2R distance from simulation
distance = @(a,b,c,d) sqrt((a-c)^2 + (b-d)^2);  %calculate distance (inline function)

r2r = zeros(size(x,2));  %nXn matrix for inter-distance
for i = 1:size(x,2)  %for first step
        for j = 1:size(x,2)
            r2r(i,j) = distance(x(1,i),y(1,i),x(1,j),y(1,j));
        end
end

r2r_tot = [r2r];

for row = 2:size(x,1)  %for rest of the steps
    r2r = zeros(size(x,2));  %nXn matrix for inter-distance
    for i = 1:size(x,2)
        for j = 1:size(x,2)
            r2r(i,j) = distance(x(row,i),y(row,i),x(row,j),y(row,j));
        end
    end
    r2r_tot = [r2r_tot; r2r];
end
            
end