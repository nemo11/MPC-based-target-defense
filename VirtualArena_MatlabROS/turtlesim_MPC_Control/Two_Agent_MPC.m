%% Using MPC controller to decrease the distance between two turtle.
%% Turtle1 moving with constant velocity, Turtle2 using control variable from MPC to minimize the given cost function.

clc; close all; clear all;

% publisher,subscriber and other required initializations....

target_data_subscriber = rossubscriber('/turtle1/pose');
target_velocity_publisher = rospublisher('/turtle1/cmd_vel');

attacker_data_subscriber = rossubscriber('/turtle2/pose');
attacker_velocity_publisher = rospublisher('/turtle2/cmd_vel');

target_Pose_Data = receive(target_data_subscriber , 10);
attacker_Pose_Data = receive(attacker_data_subscriber , 10);

dt = 1;
vm = 0.8;
vt = 0.2;

%system initialization.
realSystem = Two_Agent_RealVehicleROS(target_data_subscriber,...
                                      target_velocity_publisher,...
                                      attacker_data_subscriber,...
                                      attacker_velocity_publisher,...
                                      vm,...
                                      vt);    

%% Angle correction for turtlesim in order to constain it between -pi to pi 
theta = attacker_Pose_Data.Theta;
if( theta > 3.14 )
   theta = theta - 2*3.14;
end
if( theta < -3.14 )
   theta = theta + 2*3.14;
end
%% ........

realSystem.initialCondition = {double([attacker_Pose_Data.X;
                                attacker_Pose_Data.Y;
                                theta;
                                target_Pose_Data.X;
                                target_Pose_Data.Y;
                                sqrt((target_Pose_Data.X - attacker_Pose_Data.X)^2 + (target_Pose_Data.Y - attacker_Pose_Data.Y)^2);
                                (atan2((target_Pose_Data.Y - attacker_Pose_Data.Y),(target_Pose_Data.X - attacker_Pose_Data.X)))])};
                                                    
mpcOp = ICtMpcOp( ...
                'System'               , realSystem,...
                'HorizonLength'        , 2*dt,...
                'StageCost'            , @(t,x,u,varargin) -x(6));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
                                      'MpcOp'       , dtMpcOp ,...
                                      'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
                                       );

%% VirtualArena Object defined.                                                                      
va = VirtualArena(dtRealSystem,...
                  'StoppingCriteria'  , @(t,sysList)sqrt(((dtRealSystem.x(4)-dtRealSystem.x(1))^2+(dtRealSystem.x(5)-dtRealSystem.x(2))^2))<0.4 ,...
                  'DiscretizationStep', dt ,...
                  'RealTime',1/dt , ...
                  'PlottingStep'      , 1/dt ,...
                  'StepPlotFunction'  , @Two_Agent_PlotFunction ...
                 );
             
log = va.run();