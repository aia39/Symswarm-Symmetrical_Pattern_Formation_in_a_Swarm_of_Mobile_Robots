function [pos] = forward_withobs(vrep, dummy, bot, clientID, lmotor, rmotor,time_factor,speed,front_Sensor1,left_Sensor1,right_Sensor1, des_x,des_y)
%obstacle avoidance is employed for 3 sonar sensors

%% TANGENT BUG ALGORITHM %%
   tic

   while(1)
                [~,~,dpcenter,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor1,vrep.simx_opmode_blocking);
                [~,~,dpleft,~,~]=vrep.simxReadProximitySensor(clientID,left_Sensor1,vrep.simx_opmode_blocking);
                [~,~,dpright,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor1,vrep.simx_opmode_blocking);
                dpcenter= norm(dpcenter);
                dpleft= norm(dpleft);
                dpright= norm(dpright);
                if (dpcenter>100)
                    dpcenter=100;
                end
                if (dpleft>100)
                    dpleft=100;
                end
                if (dpright)>100
                    dpright=100;
                end
       
          dist=0.65;
          if(dpright<dist && dpright>0.01 && dpleft<dist && dpright>0.01  && dpcenter<dist && dpcenter>0.01)
                j=7
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1,vrep.simx_opmode_blocking); 
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-1,vrep.simx_opmode_blocking);
                pause(0.5)
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,0.5*speed,vrep.simx_opmode_blocking);
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-0.5*speed,vrep.simx_opmode_blocking);
                pause(.4)
               
                [~,~,dpcenter,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor1,vrep.simx_opmode_blocking);
                
                dpcenter= norm(dpcenter);
               
                if (dpcenter>100)
                    dpcenter=100;
                end
               
                if(dpcenter>0.3) %was 0.2
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed ,vrep.simx_opmode_blocking);
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                pause(.4)
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed ,vrep.simx_opmode_blocking);
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                end
                
          elseif ( dpleft<dist && dpleft>0.01 &&( dpcenter<0.14 || dpcenter> dist)  && (dpright> dist || dpright<0.17))
             j=1

             [returnCode1]=vrep.simxSetJointTargetVelocity( clientID,lmotor,1,vrep.simx_opmode_blocking); 
             [returnCode1]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-1,vrep.simx_opmode_blocking);
             pause(.4)
            [returnCode1]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed,vrep.simx_opmode_blocking);
            [returnCode1]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking); 

        
        elseif ( dpleft>dist && dpright< dist && dpright>0.01 &&( dpcenter<0.14  || dpcenter>dist))
             j=2

            [returnCode2]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1,vrep.simx_opmode_blocking); 
            [returnCode2]=vrep.simxSetJointTargetVelocity( clientID,rmotor,1,vrep.simx_opmode_blocking);
             pause(0.4)
            [returnCode2]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed,vrep.simx_opmode_blocking);
            [returnCode2]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking); 
            
         
                
       elseif((dpright>dist || dpright<0.14) && (dpleft>dist || dpleft<0.14)   && dpcenter<dist && dpcenter>0.01)
                j=3
        
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1,vrep.simx_opmode_blocking); 
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-1,vrep.simx_opmode_blocking);
                pause(0.5)
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,0.5*speed,vrep.simx_opmode_blocking);
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-0.5*speed,vrep.simx_opmode_blocking);
                pause(.4)
                
                [~,~,dpcenter,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor1,vrep.simx_opmode_blocking);
                
                dpcenter= norm(dpcenter);
               
                if (dpcenter>100)
                    dpcenter=100;
                end
               
                if(dpcenter>0.3)   %was 0.2
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed ,vrep.simx_opmode_blocking);
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                pause(.4)
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed ,vrep.simx_opmode_blocking);
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                end
                
      elseif(dpright<dist  && dpright>0.01 &&  (dpleft> dist || dpleft<0.14) && dpcenter<dist && dpcenter>0.01)
               j=5
                [returnCode4]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1,vrep.simx_opmode_blocking); 
                [returnCode4]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-1,vrep.simx_opmode_blocking);
                 pause(.3)
                [returnCode4]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1,vrep.simx_opmode_blocking); 
                [returnCode4]=vrep.simxSetJointTargetVelocity( clientID,rmotor,1,vrep.simx_opmode_blocking);
                 pause(0.4)
                
                [~,~,dpcenter,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor1,vrep.simx_opmode_blocking);
                
                dpcenter= norm(dpcenter);
               
                if (dpcenter>100)
                    dpcenter=100;
                end
               
                if(dpcenter>0.3)  %was 0.2
                [returnCode4]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed,vrep.simx_opmode_blocking);
                [returnCode4]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                pause(.3)
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed ,vrep.simx_opmode_blocking);
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                end
                
        elseif(  (dpright>dist || dpright<0.14)  && dpleft<dist   && dpleft>0.01 && dpcenter<dist && dpcenter>0.01)
         j=6
                [returnCode5]=vrep.simxSetJointTargetVelocity( clientID,lmotor,-1,vrep.simx_opmode_blocking); 
                [returnCode5]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-1,vrep.simx_opmode_blocking);
                 pause(.3)
                [returnCode5]=vrep.simxSetJointTargetVelocity( clientID,lmotor,1,vrep.simx_opmode_blocking); 
                [returnCode5]=vrep.simxSetJointTargetVelocity( clientID,rmotor,-1,vrep.simx_opmode_blocking);
                 pause(0.4)
                
                [~,~,dpcenter,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor1,vrep.simx_opmode_blocking);
                
                dpcenter= norm(dpcenter);
               
                if (dpcenter>100)
                    dpcenter=100;
                end
               
                if(dpcenter>0.3) %was 0.2
                [returnCode5]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed ,vrep.simx_opmode_blocking);
                [returnCode5]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                pause(.3)
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed ,vrep.simx_opmode_blocking);
                [returnCode7]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                end
                
          elseif(dpright<(dist) && dpright>0.01 && dpleft<(dist) && dpleft>0.01   && dpcenter> dist)
                j=4

                [~,~,dpcenter,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor1,vrep.simx_opmode_blocking);
                
                dpcenter= norm(dpcenter);
               
                if (dpcenter>100)
                    dpcenter=100;
                end
               
                if(dpcenter>0.3)  %was 0.2
                [returnCode6]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed/3,vrep.simx_opmode_blocking); 
                [returnCode6]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed/3,vrep.simx_opmode_blocking);
                 pause(0.3)
                [returnCode6]=vrep.simxSetJointTargetVelocity( clientID,lmotor,speed,vrep.simx_opmode_blocking);
                [returnCode6]=vrep.simxSetJointTargetVelocity( clientID,rmotor,speed,vrep.simx_opmode_blocking);
                end

          else  
            turning(vrep, dummy, bot, clientID, lmotor, rmotor, des_x, des_y); 
            [~,~,dpcenter,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor1,vrep.simx_opmode_blocking);
             dpcenter= norm(dpcenter);
               
             if (dpcenter>100)
                    dpcenter=100;
             end
               
             if(dpcenter>0.3)  %was 0.2
             [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lmotor,speed,vrep.simx_opmode_blocking);  %left motor will run in 0.1 speed
             [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rmotor,speed,vrep.simx_opmode_blocking);
             end
             
            pause(0.05);   %after moving fwd for 2 sec it will stop

    end
    [~,pos]=vrep.simxGetObjectPosition(clientID, bot,dummy,vrep.simx_opmode_blocking) ;
    
    last_time = toc;
   %time = last_time - tic
   if(last_time >= time_factor)   %stopping the bot after desired time
       [r1]=vrep.simxSetJointTargetVelocity(clientID,lmotor, 0,vrep.simx_opmode_blocking);  
       [r2]=vrep.simxSetJointTargetVelocity(clientID,rmotor, 0,vrep.simx_opmode_blocking); 
       break
   end
    
    
   end
   
end