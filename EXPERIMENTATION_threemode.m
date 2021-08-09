clc;
clear all;
close all;

%% Initiang Connection with Vrep
disp('Program started');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);  %needed for communication between vrep and matlab

distance = @(a,b) sqrt((a(1)-b(1))^2 + (a(2)-b(2))^2);  %calculate distance (inline function)

%% Checking For Connection
 if (clientID>-1)
        disp('Connected to remote API server');
        lmot = 'Pioneer_p3dx_leftMotor';    %for left motor
        rmot = 'Pioneer_p3dx_rightMotor';   %for right motor
        name = 'Pioneer_p3dx';
        front1= 'Pioneer_p3dx_ultrasonicSensor5';   %%for front sonar sensor (HERE WE HAVE USED 3 sonar)
        left1= 'Pioneer_p3dx_ultrasonicSensor2';
        right1= 'Pioneer_p3dx_ultrasonicSensor7';
        destination_err = 0.15;  %tolerance used as matlab can't reach exactly to a point, reduce it for more accuracy
        
        %% ******** IMPORTANT: AT FIRST CHANGE IT, SELECT NUMBER OF ROBOTS ARE BEING USED ******%%
        num_of_bots = 6;
        desired_center = [5,10];  %USER INPUT FOR DESIRED CENTER IN VREP SCENE%%
        desired_radius = 3.5;   %USER INPUT FOR DESIRED RADIUS%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        reached = zeros(1,num_of_bots);  % to keep track which robot has finished positioning in symmetrical pos(master will reach after phase 1 and follower will reach after phase 2)
        time_to_fwd = 1.5;  %increasing this will reduce time but increase probability to reach, how much time robot will move forward in each step
        speed = 2;   %speed of robot, more speed can reduce accuracy as it increase chance of collision
        
          
        [ ~,dummy]=vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_blocking);  %for reference coordinate point in vrep
        
        %%% here phase1 == stage1, phase2 == stage2 of paper %%%
        %%
        start_pos = zeros(num_of_bots, 2); %to store all starting point
        phase1 = ones(num_of_bots, 1)';  %flag variable if the robot is in first stage(reaching to destination)
        phase2 = zeros(num_of_bots, 1)';  % flag variable if the robot is in the radius to communicate with each other
        phase2_sym_pos = zeros(num_of_bots, 2);  %phase 2 symmetrical position in a vector which is sent to all robot from master
        id_taken = zeros(num_of_bots, 1)';  %flag variable to check id for the closest symmetry point don't overlap(booked id vector)
        ids = zeros(num_of_bots, 1)';  %to keep memory that which robot is reaching to which symmetric position(phase2_sym_pos)
        reach_count = 0;  %to keep count how much robot has reached the location
        
        %% EXPERIMENTAL Var(for paper's experimentation) %%
        completion_time = zeros(num_of_bots, 1)';   %FOR TIME COMPLETION
        coordinate = cell(num_of_bots,1);
        time_flag = 0;  %is 1 when we get the first phase time
        last_pos =[];
        mode = 0;   %mode = 0 if all robots start from outside of radius
        bot_mode = zeros(num_of_bots, 1)';
        posd = zeros(num_of_bots, 2);  %if robot is inside then destination
        
        %%%%%%%%%%%%%%%
        %% MAIN LOOP %%
        %%%%%%%%%%%%%%%
        for i=0:num_of_bots-1
              if(i == 0)   %in vrep for first robot name we have to use it
                % Defining Handles for each robot0
                [ ~,lmotor]=vrep.simxGetObjectHandle(clientID,lmot,vrep.simx_opmode_blocking);
                [ ~,rmotor]=vrep.simxGetObjectHandle(clientID,rmot,vrep.simx_opmode_blocking);  
                [~,front_Sensor1]=vrep.simxGetObjectHandle(clientID,front1,vrep.simx_opmode_blocking );
                [~,left_Sensor1]=vrep.simxGetObjectHandle(clientID,left1,vrep.simx_opmode_blocking );
                [~,right_Sensor1]=vrep.simxGetObjectHandle(clientID,right1,vrep.simx_opmode_blocking );
                [ ~,bot]=vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_blocking);
                [~,pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
                start_pos(i+1,:) = pos(1:2);
                pos(:,3) = [];
                coordinate{i+1} = pos;
                
                dist = distance(desired_center,pos(1:2));
                if(dist<desired_radius)
                    mode = mode + 1;
                    bot_mode(i+1) = 1; %1 means inside radius
                end
              else
                num = i-1;  %as 0 is used for null string 
                [ ~,lmotor]=vrep.simxGetObjectHandle(clientID,strcat(lmot,'#',num2str(num)),vrep.simx_opmode_blocking);
                [ ~,rmotor]=vrep.simxGetObjectHandle(clientID, strcat(rmot,'#',num2str(num)),vrep.simx_opmode_blocking);
                [~,front_Sensor1]=vrep.simxGetObjectHandle(clientID,strcat(front1,'#',num2str(num)),vrep.simx_opmode_blocking );
                [~,left_Sensor1]=vrep.simxGetObjectHandle(clientID,strcat(left1,'#',num2str(num)),vrep.simx_opmode_blocking );
                [~,right_Sensor1]=vrep.simxGetObjectHandle(clientID,strcat(right1,'#',num2str(num)),vrep.simx_opmode_blocking );
                [ ~,bot]=vrep.simxGetObjectHandle(clientID,strcat(name,'#',num2str(num)),vrep.simx_opmode_blocking);
                [~,pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
                start_pos(i+1,:) = pos(1:2);
                pos(:,3) = [];
                coordinate{i+1} = pos;
                
                dist = distance(desired_center,pos(1:2));
                if(dist<desired_radius)
                    mode = mode + 1;
                    bot_mode(i+1) = 1; %1 means inside radius
                end
                
              end
        end
        
        if(mode == num_of_bots)
            mode = 2;  %mode = 2 if all robots start within radius
        elseif(mode>0 & mode<num_of_bots)
            mode = 1;  %mode = 1 if all robots some robots start from outside of radius and some inside
        end
        
        %start_pos(:,3) = [];  %making the matrix nX2 (x,y) excluding z
        
        br = 0; %1 means break the full operation, is used to break the main while loop
        
        %%Movement and decision code
        while(~br)  %to stopping all operation if all conditions are met at the end
            for i=0:num_of_bots-1   
              if(i == 0)   %for first robot    
                %% Defining Handles (necessary for vrep) for each robot
                [ ~,lmotor]=vrep.simxGetObjectHandle(clientID,lmot,vrep.simx_opmode_blocking);
                [ ~,rmotor]=vrep.simxGetObjectHandle(clientID,rmot,vrep.simx_opmode_blocking);        
                [ ~,bot]=vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_blocking);
                [~,front_Sensor1]=vrep.simxGetObjectHandle(clientID,front1,vrep.simx_opmode_blocking );
                [~,left_Sensor1]=vrep.simxGetObjectHandle(clientID,left1,vrep.simx_opmode_blocking );
                [~,right_Sensor1]=vrep.simxGetObjectHandle(clientID,right1,vrep.simx_opmode_blocking );
                [~,~,dpcenter1,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor1,vrep.simx_opmode_blocking);
                [~,~,dpleft1,~,~]=vrep.simxReadProximitySensor(clientID,left_Sensor1,vrep.simx_opmode_blocking);
                [~,~,dpright1,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor1,vrep.simx_opmode_blocking);
                dpcenter= norm(dpcenter1);
                dpleft= norm(dpleft1);
                dpright= norm(dpright1);
                
                tic   %start time counting for robot 0
                
                if(reach_count>0)  %if 2nd phase start no matter what calculate shortest path ID to follow in 2nd phase
                %(2nd phase command)
                    id = short_pos_finder(pos, phase2_sym_pos, id_taken, ids, i, master);  %to find the shortest point ID from phase2_sym_pos vector
                    id_taken(id) = 1;  %so this id(position booked)is only assigned for this robot not valid for others
                    if(i == (master-1))
                        ids(1) = 1;  %only first position is assigned for master(I have always assigned master in first index)
                    elseif(i < master-1)
                        ids(i+2) = id;  %saving id metrics for memory that who is reaching towards where
                    else
                        ids(i+1) = id;  %so that the master index in ids can be filled by others and size remain correct(other than this it will exceed the length as master is defined at 1st pos)
                    end
                
                end
                
                %Turning and moving forward in that direction
                if(~reached(i+1) & reach_count==0)   %%for checking if robot is in phase 1 or not
                    if(~bot_mode(i+1))
                        pos = forward_withobs(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed, front_Sensor1, left_Sensor1, right_Sensor1, desired_center(1), desired_center(2));
                    else
                        [~,pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
                        posd(i+1,:) = on_radiuspt(pos, desired_center, desired_radius);
                        pos = forward_withobs(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed, front_Sensor1, left_Sensor1, right_Sensor1, posd((i+1),1), posd((i+1),2));
                    end
                    %in pos, destination is center point for a robot
                    
                    pos(:,3) = [];
                    prev_pos = coordinate{i+1};
                    pos_list =[prev_pos;pos];   %concatenating prev pos and current pos
                    coordinate{i+1} = pos_list;
                    
                elseif(~reached(i+1) & reach_count>0)  %phase 2 movement where destination is symmetrical point
                    disp(strcat('closest dist for ',num2str(i+1),' is',num2str(phase2_sym_pos(id,1)),', ',num2str(phase2_sym_pos(id,2)))); %for debugging(whether multiple robots doesn't reach toward one point)
                    
                    pos = forward_withobs(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed, front_Sensor1, left_Sensor1, right_Sensor1, phase2_sym_pos(id,1),phase2_sym_pos(id,2));
                    %in pos, destination is symmetrical point
                   
                    pos(:,3) = [];
                    prev_pos = coordinate{i+1};
                    pos_list =[prev_pos;pos];   %concatenating prev pos and current pos
                    coordinate{i+1} = pos_list;
                end
                
                if(phase1(i+1) == 1 & reach_count==0 & phase2(i+1) == 0)  %%if a robot is in phase 1 and phase 2 hasn't been initialized yet then true
                    if((~bot_mode(i+1) & distance(desired_center,pos(1:2)) - desired_radius <= destination_err) | (bot_mode(i+1) & distance(posd(i+1,:),pos(1:2)) <= destination_err))  %if the robot reach inside the radius then true
                        phase1(i+1) = 0;   %phase 1 is finished so it will be assigned to phase 2
                        phase2(i+1) = 1;
                        disp(strcat('Robot 1 reached, dist is:',num2str(distance(desired_center,pos(1:2)))));
                        reach_count = reach_count + 1;
                       
                        [ ~,bot]=vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_blocking);
                        [~,master_pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);  %acquiring master robot's position
                        master_pos = master_pos(1:2);  %excluding z axis value
                        last_pos = [master_pos];
                        %n-1 robots symmetric position around the master and it is calculated by master
                        dest = sym_pts2(num_of_bots,master_pos, desired_center);  %Argument: sym_pts(n, master_pos, center)
                        phase2_sym_pos = dest';
                        disp(phase2_sym_pos);  %for one time show all calculated symmetrical pos if there is any exception case or not
                        id_taken(1) = 1;  %first index of id_taken is always booked for master robot(I defined it)
                        ids(1) = 1;  % as first index is for master robot and it's position is in first index of phase2_sym_pos
                        master = i+1;  %to track which number of robot is master
                       
                        reached(i+1) = 1;  %master robot has finished it's work and it will stop thus reached is 1    
                    end
                    
                else     %start to work on symmetrical reaching destination (part of phase 2)
                    if(distance(phase2_sym_pos(id,:),pos(1:2)) <= destination_err)  %if this robot act as follower and if it has reached symmetric point or not
                        reached(i+1) = 1;   %robot acting as follower bot has finished the phase 2 by reaching destination
                        phase1(i+1) = 0;
                        phase2(i+1) = 1;
                        disp(strcat('Robot 1 reached, dist is: ',num2str(distance(phase2_sym_pos(id,:),pos(1:2)))));
                        reach_count = reach_count + 1;
                        last_pos = [last_pos;pos(1:2)];
                    end
                     
                end
                time = toc;
                completion_time(i+1) = completion_time(i+1)+time; %end time counting for robot0
                
                if(reach_count>0 & time_flag<1)
                    first_stage_time = completion_time(i+1);  
                    disp(strcat('First phase time: ',num2str(first_stage_time)));
                    time_flag = 1;
                end
                
                
              elseif(i>0)
                 num = i-1;  %as 0 is used for null string
                [ ~,lmotor]=vrep.simxGetObjectHandle(clientID,strcat(lmot,'#',num2str(num)),vrep.simx_opmode_blocking);
                [ ~,rmotor]=vrep.simxGetObjectHandle(clientID, strcat(rmot,'#',num2str(num)),vrep.simx_opmode_blocking);
                [ ~,bot]=vrep.simxGetObjectHandle(clientID,strcat(name,'#',num2str(num)),vrep.simx_opmode_blocking);
                [~,front_Sensor1]=vrep.simxGetObjectHandle(clientID,strcat(front1,'#',num2str(i-1)),vrep.simx_opmode_blocking );
                [~,left_Sensor1]=vrep.simxGetObjectHandle(clientID,strcat(left1,'#',num2str(i-1)),vrep.simx_opmode_blocking );
                [~,right_Sensor1]=vrep.simxGetObjectHandle(clientID,strcat(right1,'#',num2str(i-1)),vrep.simx_opmode_blocking );
                [~,~,dpcenter1,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor1,vrep.simx_opmode_blocking);
                [~,~,dpleft1,~,~]=vrep.simxReadProximitySensor(clientID,left_Sensor1,vrep.simx_opmode_blocking);
                [~,~,dpright1,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor1,vrep.simx_opmode_blocking);
                dpcenter= norm(dpcenter1);
                dpleft= norm(dpleft1);
                dpright= norm(dpright);
                
                tic  %start time counting for robots
                
                if(reach_count>0)  %if 2nd phase start no matter what calculate shortest path ID to follow in 2nd phase
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
         
                    if(~bot_mode(i+1))
                    pos = forward_withobs(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed, front_Sensor1, left_Sensor1, right_Sensor1, desired_center(1), desired_center(2));
                    else
                        [~,pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
                        posd(i+1,:) = on_radiuspt(pos, desired_center, desired_radius);
                        pos = forward_withobs(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed, front_Sensor1, left_Sensor1, right_Sensor1, posd((i+1),1), posd((i+1),2));
                    end
                    
                    pos(:,3) = [];
                    prev_pos = coordinate{i+1};
                    pos_list =[prev_pos;pos];   %concatenating prev pos and current pos
                    coordinate{i+1} = pos_list;
                    
                elseif(~reached(i+1) & reach_count>0)  %phase 2 movement where destination of robot is symmetric point
                    disp(strcat('closest dist for ',num2str(i+1),' is',num2str(phase2_sym_pos(id,1)),', ',num2str(phase2_sym_pos(id,2)))); %for debugging
                    
                    pos = forward_withobs(vrep, dummy, bot, clientID, lmotor, rmotor, time_to_fwd, speed, front_Sensor1, left_Sensor1, right_Sensor1, phase2_sym_pos(id,1),phase2_sym_pos(id,2));
                    
                    pos(:,3) = [];
                    prev_pos = coordinate{i+1};
                    pos_list =[prev_pos;pos];   %concatenating prev pos and current pos
                    coordinate{i+1} = pos_list;
                    
                end
                
                if(phase1(i+1) == 1 & reach_count==0 & phase2(i+1) == 0) %check whether it is in 1st phase or not
                    if((~bot_mode(i+1) & distance(desired_center,pos(1:2)) - desired_radius <= destination_err) | (bot_mode(i+1) & distance(posd(i+1,:),pos(1:2)) <= destination_err))
                        disp(strcat('robot ',num2str(i+1),'reached, dist is:',num2str(distance(desired_center,pos(1:2)))));
                        reach_count = reach_count + 1;
                        phase1(i+1) = 0;
                        phase2(i+1) = 1;
                       
                        num = i-1;  %as 0 is used for null string
                        [ ~,bot]=vrep.simxGetObjectHandle(clientID,strcat(name,'#',num2str(num)),vrep.simx_opmode_blocking);
                        [~,master_pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
                        master_pos = master_pos(1:2);
                        last_pos = [master_pos];
                        dest = sym_pts2(num_of_bots,master_pos, desired_center);  %sym_pts(n, master_pos, center) , to get symmetric position around master
                        phase2_sym_pos = dest';
                        disp(phase2_sym_pos);
                        id_taken(1) = 1;   %1st index is for master robot
                        ids(1) = 1;  %1st index is for master robot
                        master = i+1;   %to keep track which robot is master
                      
                        reached(i+1) = 1;  %end of master robot's job and it will stop
                    end
                
                else     %start to work on symmetrical reaching destination
                    if(distance(phase2_sym_pos(id,:),pos(1:2)) <= destination_err)  %if follower robot has reached the symmetric point or not
                        reached(i+1) = 1;  
                        phase1(i+1) = 0;
                        phase2(i+1) = 1;
                        disp(strcat('Robot',num2str(i+1),'reached, dist is:',num2str(distance(phase2_sym_pos(id,:),pos(1:2)))));
                        reach_count = reach_count + 1;
                        last_pos = [last_pos;pos(1:2)];
                    end    
                end
                
                time = toc;
                completion_time(i+1) = completion_time(i+1)+time; %ending count on each epoch 
                %for experimentation to get first phase time
                if(reach_count>0 & time_flag<1)
                    first_stage_time = completion_time(i+1);  
                    disp(strcat('First phase time:',num2str(first_stage_time)));
                    time_flag = 1;
                end
                
              if(all(reached) == 1)  %if all robot has reached to the symmetric position or not
                  disp('Finished phase 1&2');
                  br = 1;   %br = 1 means the main while loop will turn off and whole code will turn off
              end
            end
        end
        end
     %% experimentation data   
     completion_time
     [x,y] = draw_2d(coordinate,desired_center);  %drawing 2d coordinate graph for trajectory (PLEASE CHECK IT BEFORE RUNNING CODE)
     inter_dist = r2r_local(x,y);  %interdistance for each robot after each step, every 4 rows represent each step interdistance matrix
     [error, theoretical_dist, simulated_avg_dist] = r2r_error(x, y, num_of_bots, desired_radius)  %r2r error(standard deviation based error)
     first_stage_time   %it also can be the completion time of master robot 
     %inter_dist     %to get inter-distance matrix, number of step = dim(1)/num_of_bot
     last_pos
     %%
     vrep.simxFinish(-1);
 else
       disp('Failed connecting to remote API server');  
 vrep.delete();
 
 end