function [] = turning(vrep, dummy, bot, clientID, lmotor, rmotor, des_x,des_y)
    %% Calling A-star for path Planning
     % Getting Initial position of turtlebot from vrep environment
     [~,pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking);
     
     % Getting Orientation
     [~,theta]=vrep.simxGetObjectOrientation(clientID, bot,dummy,vrep.simx_opmode_blocking);
     %   Calculationg Orientation from Path
     newxtheta=atan2((des_y - pos(2)),(des_x - pos(1)));

     w=1; % Setting roatation velocity of wheels. (in rad/s)  %for reducing time set w=2
     k=1; % variable for while loop

     %% Setting orientation
        %disp('Turning');
    %         [ r12, tt(lt)]=vrep.simxGetFloatSignal( clientID,'Turtlebot2_simulation_time',vrep.simx_opmode_blocking);lt=lt+1;
     while(k==1)
        diff=  abs(abs(theta(3))-abs(newxtheta));
       if(diff<0.04)
           break
       end
       if (theta(3)<newxtheta)
           % taking left
           %disp('Turning left');
           ul=-w;ur=w;
           [r1]=vrep.simxSetJointTargetVelocity( clientID,lmotor, ul,vrep.simx_opmode_blocking);  
           [r2]=vrep.simxSetJointTargetVelocity( clientID,rmotor, ur,vrep.simx_opmode_blocking); 
       else
           %taking right
           ul=w;ur=-w;
           %disp('Turning right');
           [r3]=vrep.simxSetJointTargetVelocity( clientID,lmotor, ul,vrep.simx_opmode_blocking);  
           [r4]=vrep.simxSetJointTargetVelocity( clientID,rmotor, ur,vrep.simx_opmode_blocking); 
       end
       [~,theta]=vrep.simxGetObjectOrientation(clientID, bot,dummy,vrep.simx_opmode_blocking);

     end
      [~,theta]=vrep.simxGetObjectOrientation(clientID, bot,dummy,vrep.simx_opmode_blocking);
      %disp('Done Turning');

      % Once Orientation set, angular velocities are returned back to zero
     [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor, 0,vrep.simx_opmode_blocking);  
     [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor, 0,vrep.simx_opmode_blocking);

end