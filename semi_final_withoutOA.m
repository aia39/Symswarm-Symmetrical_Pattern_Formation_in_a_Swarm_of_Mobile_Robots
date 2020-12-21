clc;
clear all;
close all;

% Initiang Connection with Vrep
disp('Program started');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

distance = @(a,b) sqrt((a(1)-b(1))^2 + (a(2)-b(2))^2);  %calculate distance (inline function)

%% Checking For Connection
 if (clientID>-1)
        disp('Connected to remote API server');
        lmot = 'Pioneer_p3dx_leftMotor';
        rmot = 'Pioneer_p3dx_rightMotor';
        name = 'Pioneer_p3dx';
        destination_err = 0.15;
        num_of_bots = 4;
        reached = zeros(1,num_of_bots);
        time_to_fwd = 3.5;  %increasing this will reduce time but increase probability to reach
        speed = 2.5;
        desired_center = [3,3.7];
        desired_radius = 2;
        reach_count = 0;

        [ ~,dummy]=vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_blocking);
        
        start_pos = zeros(num_of_bots, 3);
        phase1 = ones(num_of_bots, 1)';  %flag variable if the robot is in first stage(reaching to destination)
        phase2 = zeros(num_of_bots, 1)';  % flag variable if the robot is in the radius to communicate with each other
        reached_radius = zeros(num_of_bots, 1)';
        phase2_sym_pos = zeros(num_of_bots, 2);  %phase 2 symmetrical position
        id_taken = zeros(num_of_bots, 1)';  %flag variable to check id for the closest symmetry point not overlap
        ids = zeros(num_of_bots, 1)';  %to keep memory that which robot is reaching to which symmetric position
       
        
        %phase2_sym_pos = [3,4;8,9;4.5,5];   %%for testing purpose
        
        for i=0:num_of_bots-1
              if(i == 0)
                %% Defining Handles for each robot0
                [ ~,lmotor]=vrep.simxGetObjectHandle(clientID,lmot,vrep.simx_opmode_blocking);
                [ ~,rmotor]=vrep.simxGetObjectHandle(clientID,rmot,vrep.simx_opmode_blocking);        
                [ ~,bot]=vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_blocking);
                [~,pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
                start_pos(i+1,:) = pos;
              else
                num = i-1;  %as 0 is used for null string
                [ ~,lmotor]=vrep.simxGetObjectHandle(clientID,strcat(lmot,'#',num2str(num)),vrep.simx_opmode_blocking);
                [ ~,rmotor]=vrep.simxGetObjectHandle(clientID, strcat(rmot,'#',num2str(num)),vrep.simx_opmode_blocking);
                [ ~,bot]=vrep.simxGetObjectHandle(clientID,strcat(name,'#',num2str(num)),vrep.simx_opmode_blocking);
                [~,pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
                start_pos(i+1,:) = pos;
              end
        end
        
        start_pos(:,3) = [];  %making the matrix nX2 (x,y) excluding z
        %any_desired_center = [3,3];
        
        br = 0; %1 means break the full operation
        
        %%movement
        while(~br)
            for i=0:num_of_bots-1
              if(i == 0 )%& ~reached(i+1))
                %% Defining Handles for each robot01
                [ ~,lmotor]=vrep.simxGetObjectHandle(clientID,lmot,vrep.simx_opmode_blocking);
                [ ~,rmotor]=vrep.simxGetObjectHandle(clientID,rmot,vrep.simx_opmode_blocking);        
                [ ~,bot]=vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_blocking);

                if(reach_count>0)  %if 2nd phase start no matter what calculate shortest path ID to follow in 2nd phase
                %%find shortest distance using    short_pos_finder()
                id = short_pos_finder(pos, phase2_sym_pos, id_taken, ids, i, master);
                id_taken(id) = 1;  %so this id(position booked)is only assigned for this robot not valid for others
                if(i == (master-1))
                    ids(1) = 1;  %only first position is assigned for master
                elseif(i < master-1)
                    ids(i+2) = id;  %saving id metrics for memory that who is reaching towards where
                else
                    ids(i+1) = id;  %so that the master index can be filled by others and size remain concise
                end
                
                end
  
                
                %Turning and moving forward in that direction
                if(~reached(i+1) & reach_count==0) %all(phase2_sym_pos) == 0) %%phase 1 movement where destination is center point
                    turning(vrep, dummy, bot, clientID, lmotor, rmotor, desired_center(1),desired_center(2));  %turn from current pos to destination
                    pos = forward(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed);
                
                elseif(~reached(i+1) & reach_count>0)  %phase 2 movement where destination is symmetrical point
                    
                    %%find shortest distance using                     
                    %disp(strcat('closest id for',num2str(i),'is',num2str(id)));
                    disp(strcat('closest dist for',num2str(i+1),'is',num2str(phase2_sym_pos(id,1)),num2str(phase2_sym_pos(id,2)))); %for debugging
                    turning(vrep, dummy, bot, clientID, lmotor, rmotor, phase2_sym_pos(id,1),phase2_sym_pos(id,2));  %%shortest co ordinate instead of phase2_sym_pos %turn from current pos to destination
                    pos = forward(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed);
                    %phase2_sym_pos(id,:) = [];  %to remove from priority list so that no conflict occurs(not needed)
                end
                
                if(phase1(i+1) == 1 & reach_count==0 & phase2(i+1) == 0) %all(phase2_sym_pos) == 0)  %%if a robot is in phase 1 and phase 2 hasn't been initialized yet
                    if(distance(desired_center,pos(1:2)) - desired_radius <= destination_err)
                        %reached(i+1) = 1;
                        phase1(i+1) = 0;
                        phase2(i+1) = 1;
                        disp(strcat('Robot 1 reached, dist is:',num2str(distance(desired_center,pos(1:2)))));
                        %robot_reach(1) = 1;
                        reach_count = reach_count + 1;
                        reached_radius(i+1) = 1;
                        
                        %if(reach_count >0) %it means the first one to reach radius and it will act as master
                        [ ~,bot]=vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_blocking);
                        [~,master_pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
                        master_pos = master_pos(1:2);
                        %dest = get_symmetry_Nmin1(master_pos, desired_center, num_of_bots); %n-1 robots symmetric position calculated by master
                        dest = sym_pts(num_of_bots,master_pos, desired_center);  %sym_pts(n, master_pos, center) , to get symmetric position around master
                        dest(:,end) = [];  %deleting last indices as it is added in subsequent line
                        phase2_sym_pos = [master_pos;dest'];  %concatenate
                        disp(phase2_sym_pos);
                        id_taken(1) = 1;
                        ids(1) = 1;
                        master = i+1;
                        %disp('Master on');
                        
                        %phase2_sym_pos(i+1,:) = master_pos;  %need to uncomment
                        reached(i+1) = 1;
                        %end
                    end
                    
                else     %start to work on symmetrical reaching destination
                    if(distance(phase2_sym_pos(id,:),pos(1:2)) <= destination_err)
                        reached(i+1) = 1;
                        phase1(i+1) = 0;
                        phase2(i+1) = 1;
                        disp(strcat('Robot 1 reached, dist is:',num2str(distance(phase2_sym_pos(id,:),pos(1:2)))));
                        %robot_reach(1) = 1;
                        reach_count = reach_count + 1;
                        %reached_radius(i+1) = 1;
                    end
                     
                end
                
              elseif(i>0)%& ~reached(i+1))
                 num = i-1;  %as 0 is used for null string
                [ ~,lmotor]=vrep.simxGetObjectHandle(clientID,strcat(lmot,'#',num2str(num)),vrep.simx_opmode_blocking);
                [ ~,rmotor]=vrep.simxGetObjectHandle(clientID, strcat(rmot,'#',num2str(num)),vrep.simx_opmode_blocking);
                [ ~,bot]=vrep.simxGetObjectHandle(clientID,strcat(name,'#',num2str(num)),vrep.simx_opmode_blocking);
                
                if(reach_count>0)  %if 2nd phase start no matter what calculate shortest path ID to follow in 2nd phase
                %%find shortest distance using    short_pos_finder()
                id = short_pos_finder(pos, phase2_sym_pos, id_taken, ids, i, master);
                id_taken(id) = 1;   %so this id is only assigned for this robot not valid for others
                if(i == (master-1))
                    ids(1) = 1;
                elseif(i < master-1)
                    ids(i+2) = id;  %saving id metrics for memory that who is reaching towards where
                else
                    ids(i+1) = id;  %so that ids not exceed it's limit
                end
                end
                
                
                %Turning and moving forward in that direction
                if(~reached(i+1)& reach_count==0)
          
                    turning(vrep, dummy, bot, clientID, lmotor, rmotor, desired_center(1),desired_center(2));  %turn from current pos to destination  
                    pos = forward(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed);
                
                elseif(~reached(i+1) & reach_count>0)  %phase 2 movement where destination is symmetrical point
                    %disp(strcat('closest id for',num2str(i+1),'is',num2str(id)));
                    disp(strcat('closest dist for',num2str(i+1),'is',num2str(phase2_sym_pos(id,1)),num2str(phase2_sym_pos(id,2)))); %for debugging
                    turning(vrep, dummy, bot, clientID, lmotor, rmotor, phase2_sym_pos(id,1),phase2_sym_pos(id,2));  %turn from current pos to destination
                    pos = forward(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed);
                end
                
                if(phase1(i+1) == 1 & reach_count==0 & phase2(i+1) == 0) %all(phase2_sym_pos) == 0)
                    if(distance(desired_center,pos(1:2)) - desired_radius <= destination_err)
                        %reached(i+1) = 1;
                        disp(strcat('robot ',num2str(i+1),'reached, dist is:',num2str(distance(desired_center,pos(1:2)))));
                        %robot_reach(i+1) = 1;
                        reach_count = reach_count + 1;
                        phase1(i+1) = 0;
                        phase2(i+1) = 1;
                        reached_radius(i+1) = 1;
                        %scatter(desired(i,1),desired(i,2),100,'filled','g')
                        
                        %if(reach_count >0) %it means the first one to reach radius and it will act as master
                        num = i-1;  %as 0 is used for null string
                        [ ~,bot]=vrep.simxGetObjectHandle(clientID,strcat(name,'#',num2str(num)),vrep.simx_opmode_blocking);
                        [~,master_pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
                        master_pos = master_pos(1:2);
                        %dest = get_symmetry_Nmin1(master_pos, desired_center, num_of_bots); %n-1 robots symmetric position calculated by master
                        dest = sym_pts(num_of_bots,master_pos, desired_center);  %sym_pts(n, master_pos, center) , to get symmetric position around master
                        dest(:,end) = [];  %deleting last indices as it is added in subsequent line
                        phase2_sym_pos = [master_pos;dest'];  %concatenate(master_pos should be concatenate into i+1 rd indices 
                        disp(phase2_sym_pos);
                        %%%akib you need to insert master_pos into i+1 th
                        %%%row(fix it)
                        id_taken(1) = 1;   %1st index is for master robot
                        ids(1) = 1;  %1st index is for master robot
                        master = i+1;
                        
                        %disp('master is on');
                        
                        %phase2_sym_pos(i+1,:) = master_pos;   %need to uncomment it
                        reached(i+1) = 1;
                        %end
                    end
%                 elseif(phase2(i+1) == 1)% & ~reached(i+1))   %%NOTE: If we want to keep communication on in phase 2 with master then comment out reached(i+1)
                
                else     %start to work on symmetrical reaching destination
                    if(distance(phase2_sym_pos(id,:),pos(1:2)) <= destination_err)
                        reached(i+1) = 1;
                        phase1(i+1) = 0;
                        phase2(i+1) = 1;
                        disp(strcat('Robot',num2str(i+1),'reached, dist is:',num2str(distance(phase2_sym_pos(id,:),pos(1:2)))));
                        %robot_reach(1) = 1;
                        reach_count = reach_count + 1;
                        %reached_radius(i+1) = 1;
                    end
                    
                end
                %now find error in angle
                %then use P controller
                %then correct angle
              if(all(reached) == 1)
                  disp('Finished phase 1&2');
                  br = 1;
              end
            end
        end
     end
     vrep.simxFinish(-1);
 else
       disp('Failed connecting to remote API server');  
 vrep.delete();
 
 end