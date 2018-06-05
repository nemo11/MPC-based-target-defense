%% Using MPC controller to decrease the distance between two turtle.
%% Turtle1 moving with constant velocity, Turtle2 using control variable from MPC to minimize the given cost function.

clc; close all; clear all;

% publisher,subscriber and other required initializations....
%%
attacker_pose_subscriber = rossubscriber('attacker/mavros/global_position/global');
attacker_compass_subscriber = rossubscriber('attacker/mavros/global_position/compass_hdg');
attacker_velocity_publisher = rospublisher('attacker/mavros/setpoint_velocity/cmd_vel_unstamped');

target_pose_subscriber = rossubscriber('target/mavros/global_position/global');
target_compass_subscriber = rossubscriber('target/mavros/global_position/compass_hdg');
target_velocity_publisher = rospublisher('target/mavros/setpoint_velocity/cmd_vel_unstamped');

target_pose_data = receive(target_pose_subscriber , 10);
target_compass_data = receive(target_compass_subscriber , 10);
attacker_pose_data = receive(attacker_pose_subscriber , 10);
attacker_compass_data = receive(attacker_compass_subscriber , 10);

dt = 0.1;
vm = 8;
vt = 2;
%%
%system initialization.
realSystem = Two_Agent_SITL_RealVehicleROS(target_pose_subscriber,...
                                      target_compass_subscriber,...
                                      target_velocity_publisher,...
                                      attacker_pose_subscriber,...
                                      attacker_compass_subscriber,...
                                      attacker_velocity_publisher,...
                                      vm,...
                                      vt);    

%% Angle correction for turtlesim in order to constain it between -pi to pi 
theta = 3.14*(attacker_compass_data.Data)/180;
if( theta > 3.14 )
   theta = theta - 2*3.14;
end

%% ........

realSystem.initialCondition = {double([attacker_pose_data.Latitude;
                                attacker_pose_data.Longitude;
                                theta;
                                target_pose_data.Latitude;
                                target_pose_data.Longitude;
                                sqrt((target_pose_data.Latitude - attacker_pose_data.Latitude)^2 + (target_pose_data.Longitude - attacker_pose_data.Longitude)^2);
                                (atan2((target_pose_data.Longitude - attacker_pose_data.Longitude),(target_pose_data.Latitude - attacker_pose_data.Latitude)))])};
                                                    
mpcOp = ICtMpcOp( ...
                'System'               , realSystem,...
                'HorizonLength'        , dt,...
                'StageCost'            , @(t,x,u,varargin) -x(6));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
                                      'MpcOp'       , dtMpcOp ,...
                                      'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
                                       );

%% VirtualArena Object defined.                                                                      
va = VirtualArena(dtRealSystem,...
                  'StoppingCriteria'  , @(t,sysList)sqrt(((dtRealSystem.x(4)-dtRealSystem.x(1))^2+(dtRealSystem.x(5)-dtRealSystem.x(2))^2))<0.0004 ,...
                  'DiscretizationStep', 2*dt ,...
                  'RealTime',1/dt , ...
                  'PlottingStep'      , 1/dt ,...
                  'StepPlotFunction'  , @Two_Agent_PlotFunction ...
                 );
             
log = va.run();
