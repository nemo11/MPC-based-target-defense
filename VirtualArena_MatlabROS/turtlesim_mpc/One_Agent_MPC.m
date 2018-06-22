clc; close all; clear all;

attacker_data_subscriber = rossubscriber('/turtle1/pose');
attacker_velocity_publisher = rospublisher('/turtle1/cmd_vel');

attacker_pose_data = receive(attacker_data_subscriber , 10);

dt = 1;
vm = 0.5;

XT = input('target X :');
YT = input('target Y :');
target = [XT , YT];

%system initialization.
realSystem = One_Agent_RealVehicleROS(attacker_data_subscriber,...
                                      attacker_velocity_publisher,...
                                      vm,...
                                      target);    

%% Angle correction for turtlesim in order to constain it between -pi to pi 
theta = attacker_pose_data.Theta;
if( theta > 3.14 )
   theta = theta - 2*3.14;
end
if( theta < -3.14 )
   theta = theta + 2*3.14;
end
%% ........

realSystem.initialCondition = {double([attacker_pose_data.X;
                                attacker_pose_data.Y;
                                theta;
                                XT;
                                YT;
                                sqrt((XT - attacker_pose_data.X)^2 + (YT - attacker_pose_data.Y)^2);
                                (atan2((YT - attacker_pose_data.Y),(XT - attacker_pose_data.X)))])};
                                                    
mpcOp = ICtMpcOp( ...
                'System'               , realSystem,...
                'HorizonLength'        , 2*dt,...
                'StageCost'            , @(t,x,u,varargin) x(6));
            

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
                                      'MpcOp'       , dtMpcOp ,...
                                      'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
                                       );

%% VirtualArena Object defined.                                                                      
va = VirtualArena(dtRealSystem,...
                  'StoppingCriteria'  , @(t,sysList)sqrt(((dtRealSystem.x(4)-dtRealSystem.x(1))^2+(dtRealSystem.x(5)-dtRealSystem.x(2))^2))<0.1 ,...
                  'DiscretizationStep', dt ,...
                  'RealTime',1/dt); 
                              
va.run();