%% Using MPC controller to decrease the distance between two turtle.

% Target moving with constant velocity
% Attacker using control variable from MPC to minimize the given cost function.

clc; close all; clear all;

% publisher,subscriber and other required initializations....

attacker_LatLon_subscriber = rossubscriber('/attacker/mavros/global_position/global');
attacker_angle_subscriber = rossubscriber('/attacker/mavros/global_position/compass_hdg');
attacker_velocity_publisher = rospublisher('/attacker/mavros/setpoint_velocity/cmd_vel_unstamped');

target_LatLon_subscriber = rossubscriber('/target/mavros/global_position/global');
target_angle_subscriber = rossubscriber('/target/mavros/global_position/compass_hdg');
target_velocity_publisher = rospublisher('/target/mavros/setpoint_velocity/cmd_vel_unstamped');

target_LatLon = receive(target_LatLon_subscriber , 10);
attacker_LatLon = receive(attacker_LatLon_subscriber , 10);
attacker_angle = receive(attacker_angle_subscriber , 10);

dt = 1;
vm = 0.5;
vt = 0.3;

%system initialization.
realSystem = two_agent_rover_system_ROS(target_LatLon_subscriber,...
                                                    target_angle_subscriber,...
                                                    target_velocity_publisher,...
                                                    attacker_LatLon_subscriber,...
                                                    attacker_angle_subscriber,...
                                                    attacker_velocity_publisher,...
                                                    vm,...
                                                    vt);    

%% bounding theta of rover between -pi to pi

 attacker_angle.Data = 90 - attacker_angle.Data;

%% Convert to UTM

[attacker_utmX, attacker_utmY] = deg2utm(attacker_LatLon.Latitude, attacker_LatLon.Longitude);
[target_utmX, target_utmY] = deg2utm(target_LatLon.Latitude, target_LatLon.Longitude);

%% ........

realSystem.initialCondition = {[attacker_utmX;
                                attacker_utmY;
                                deg2rad(attacker_angle.Data);
                                target_utmX;
                                target_utmY;
                                sqrt((target_utmX - attacker_utmX)^2 + (target_utmY - attacker_utmY)^2);
                                (atan2((target_utmY - attacker_utmY),(target_utmX - attacker_utmX)))]};
                                                    
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
                  'StoppingCriteria'    , @(t,sysList)sqrt(((dtRealSystem.x(4)-dtRealSystem.x(1))^2+(dtRealSystem.x(5)-dtRealSystem.x(2))^2))<0.00004 ,...
                  'DiscretizationStep'  , dt ,...
                  'RealTime'            , 1/dt , ...
                  'PlottingStep'        , 1/dt ...
                 );
             
log = va.run();