function[id] = short_pos_finder(cur_pos, dest_pos, id_booked, ids, num, master)
%%%%THIS FUNCTION HELPS TO FIND NEAREST POINT FOR EACH ROBOT (Algorithm 2 of paper) 
%num means 'i' th robot

distance = @(a,b) sqrt((a(1)-b(1))^2 + (a(2)-b(2))^2);  %calculate distance (inline function)

%%it will activate after once all indices of the id_booked matrices is filled up
if(all(id_booked))
    if(num == (master-1))
        id = 1;  %master will always take first index
    elseif(num < master-1)
        id = ids(num+2);
    else
        id = ids(num+1);  %it will assign previously assigned ids to corresponding robot if all ids is booked
    end
    
else  
    %if any of them is not booked then do further calculation
    dist = zeros(1, length(dest_pos-1));

    for i = 2:length(dest_pos)  %starting from 2nd index as first index is booked for master so we don't want it to use by other
        dist(i) = distance(cur_pos, dest_pos(i,:));   %determining distance metrics between points
    end

    for j = 2:length(id_booked)
      if(~id_booked(j))
        id = j;
        shortest = dest_pos(id);  %after selecting the first valid id for comparison
        break
      else 
        continue
      end
    end

    
    for j = 2:length(id_booked)
      if(~id_booked(j))
        if(dist(j)<shortest)
           shortest = dist(j);
           id = j;
        end
      else   %if id_booked is for other robots then it will skip it from calulation
          continue
      end
    end

end

end