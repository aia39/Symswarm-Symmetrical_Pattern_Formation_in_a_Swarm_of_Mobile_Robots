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
if (n==1)
     plot(X,Y,'LineWidth',3,'Marker','s');



elseif (n==2)
    plot(X,Y,'LineWidth',3,'Marker','o');

  
elseif (n==3)
    plot(X,Y,'LineWidth',3,'Marker','+');
   

elseif (n==4)
     %plot(X,Y,'-v','MarkerIndices',1:10:length(Y));
     plot (X,Y,'-v')
     elseif (n==5)
     %plot(X,Y,'-^','MarkerIndices',1:10:length(Y));
     plot(X,Y,'-^');
     elseif (n==6)
     %plot(X,Y,'-<','MarkerIndices',1:10:length(Y));
     plot(X,Y,'-<');
 end
    
hold on
xlabel('X Position');
ylabel('Y Position');
axis equal
 axis([0 15 0 15])

scatter(5,10,50,'filled')
%  hold on                                                 // For obstacles, uncomment 62-69 
% scatter(8.30,10.15,224,'s','filled','k')
% hold on
% scatter(7.1808 , 8.5504,218,'s','filled','k')
% hold on
% scatter(7.2018 , 11.8500,224,'s','filled','k')
% hold on
% scatter(5.8028 ,10.2253,224,'s','filled','k')

%%need to change if n~=3
%legend is written for 6 robots, you may need to change it if number of robot is different
legend({'1st Robot','2nd Robot','3rd Robot','4th Robot','5th Robot','6th Robot','Center' },'Location','southeast')  %change this according to number of robot

for i = 1:length(coordinate)
    scatter(X(1,i),Y(1,i),120,'x');  %starting pos marker
    scatter(X(end,i),Y(end,i),120,'o')  %ending pos marker
end


end