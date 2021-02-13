function [X,Y]= draw_2d(coordinate,center)
maximum = max(cellfun('length',coordinate));
for n=1:length(coordinate)
    a = coordinate{n};
    if(length(a)<maximum)
        temp = a;
        req = maximum-length(a);
        for i = 1:req
            temp = [temp;a(end,:)];
        end
    coordinate{n} = temp;
    end
end


a = coordinate{1};
X = [a(:,1)];
Y = [a(:,2)];
for n=2:length(coordinate)
    a = coordinate{n};
    X = [X a(:,1)];  %appending
    Y = [Y a(:,2)];  %appending
end

plot(X,Y,'LineWidth',3);
%title('');
hold on
xlabel('X Position');
ylabel('Y Position');
axis([-1 inf 0 10])

scatter(center(1),center(2),50,'filled')

legend({'1st Robot','2nd Robot','3rd Robot','Center'},'Location','southwest')  %change this according to number of robot

for i = 1:length(coordinate)
    scatter(X(1,i),Y(1,i),120,'x');  %starting pos marker
    scatter(X(end,i),Y(end,i),120,'o')  %ending pos marker
end


end