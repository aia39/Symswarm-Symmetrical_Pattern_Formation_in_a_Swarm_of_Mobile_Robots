function [error, theoretical_dist, simulated_mean_dist, cumulative_err]  = r2r_error(x, y, n, r)
%parameters 
%x = x coordinate of each step for all bots, y = x coordinate of each step for all bots
%n = number of robots, r = desired radius

distance = @(a,b,c,d) sqrt((a-c)^2 + (b-d)^2);  %calculate distance (inline function)
theoretical_dist = r2r_distance(n,r);  %inter-distance from the equation
x_last = x(end,:);  %taking the last destination coordinate
y_last = y(end,:);    
    

for i = 1:size(x,2)-1
     r2r(i) = distance(x_last(i),y_last(i),x_last(i+1),y_last(i+1));
end

r2r(size(x,2)) = distance(x_last(1),y_last(1),x_last(end),y_last(end));  %distance between first and last index which is assigned to last index of the r2r_error


simulated_mean_dist = mean(r2r);
e = abs(r2r - theoretical_dist);  %r2r = simulated r2r distance from euclidean distance, e = difference betn simulated and theoretical
cumulative_err = sum(e);
error = std(e, 0, 2);  % error is the standard deviation from all robots absolute error, we can get idea how much overall system deviates

end