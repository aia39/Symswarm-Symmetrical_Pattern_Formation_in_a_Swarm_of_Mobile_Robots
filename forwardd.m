function [pos] = forwardd(vrep, dummy, bot, clientID, lmotor, rmotor,time_factor,speed,dpcenter,dpleft,dpright, des_x,des_y)
     
        dist=0.6;
        
            if ( dpleft<dist && dpleft>0.001  && dpright> dist  && dpcenter> dist)
            j=1
            [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,1.5,vrep.simx_opmode_blocking); 
            [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-1.5,vrep.simx_opmode_blocking);
             pause(1.2)
            [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed,vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking); 

        
        elseif ( dpleft>dist && dpright< dist && dpright>0.001  && dpcenter>dist)
            j=2
            [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1.5,vrep.simx_opmode_blocking); 
            [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,1.5,vrep.simx_opmode_blocking);
             pause(1.2)
            [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed,vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking); 
            
        elseif (dpright<dist && dpright>0.001 && dpleft<dist && dpright>0.001    && dpcenter<dist && dpcenter>0.001)
            j=7
                [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1.5,vrep.simx_opmode_blocking); 
                [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-1.5,vrep.simx_opmode_blocking);
                 pause(1.5)
                [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-2/3*speed,vrep.simx_opmode_blocking);
                [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,2/3*speed,vrep.simx_opmode_blocking);
                pause(1.4)
                 [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed,vrep.simx_opmode_blocking);
                [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                
       elseif(dpright>dist && dpleft>dist   && dpcenter<dist && dpcenter>0.001)
             j=3
                [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1.5,vrep.simx_opmode_blocking); 
                [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,1.5,vrep.simx_opmode_blocking);
                 pause(1.2)
                [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,1.5,vrep.simx_opmode_blocking);
                [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,1.5,vrep.simx_opmode_blocking);
                
        elseif(dpright<(dist) && dpright>0.001 && dpleft<(dist) && dpleft>0.001   && dpcenter> dist)
               j=4
               [r]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-speed/3,vrep.simx_opmode_blocking); 
            [r]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-speed/3,vrep.simx_opmode_blocking);
             pause(0.4)
            [r]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed/3,vrep.simx_opmode_blocking); 
            [r]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed/3,vrep.simx_opmode_blocking);
             pause(1.4)
            [r]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed,vrep.simx_opmode_blocking);
            [r]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
       
            
      elseif(dpright<dist  && dpright>0.001 &&  dpleft> dist && dpcenter<dist && dpcenter>0.001)
               j=5
                [r]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1.5,vrep.simx_opmode_blocking); 
                [r]=vrep.simxSetJointTargetVelocity( clientID,rmotor,1.5,vrep.simx_opmode_blocking);
                 pause(1.4)
                [r]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed,vrep.simx_opmode_blocking);
                [r]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                
     elseif(dpright>dist  && dpleft<dist   && dpleft>0.001 && dpcenter<dist && dpcenter>0.001)
         j=6
                [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-0.5,vrep.simx_opmode_blocking); 
                [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-0.5,vrep.simx_opmode_blocking);
                 pause(0.9)
                [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,1.5,vrep.simx_opmode_blocking); 
                [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-1.5,vrep.simx_opmode_blocking);
                 pause(1.6)
                [~]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed ,vrep.simx_opmode_blocking);
                [~]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed ,vrep.simx_opmode_blocking);
            else 
            turning(vrep, dummy, bot, clientID, lmotor, rmotor, des_x, des_y); 
            [~]=vrep.simxSetJointTargetVelocity(clientID,lmotor,speed,vrep.simx_opmode_blocking);  %left motor will run in 0.1 speed
            [~]=vrep.simxSetJointTargetVelocity(clientID,rmotor,speed,vrep.simx_opmode_blocking); 
        end
        
    pause(time_factor);   %after moving fwd for 2 sec it will stop
    %stop
    [~]=vrep.simxSetJointTargetVelocity(clientID,lmotor,0.0,vrep.simx_opmode_blocking);  %left motor will run in 0.1 speed
    [~]=vrep.simxSetJointTargetVelocity(clientID,rmotor,0.0,vrep.simx_opmode_blocking);  %right motor will run in 0.1 speed   
    %disp('Forwarding finish');
    [~,pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking) ;
    
    
end