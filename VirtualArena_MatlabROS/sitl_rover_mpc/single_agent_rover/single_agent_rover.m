%% Using MPC controller to decrease the distance between rover and target.

% Turtle1 using control variable from MPC to minimize the given cost function.

clc; close all; clear all;

% publisher,subscriber and other required initializations....

LatLon_subscriber = rossubscriber('attacker/mavros/global_position/global');
angle_subscriber = rossubscriber('attacker/mavros/global_position/compass_hdg');
velocity_publisher = rospublisher('attacker/mavros/setpoint_velocity/cmd_vel_unstamped');

LatLon_Msg = receive(LatLon_subscriber, 10);
angle_Msg = receive(angle_subscriber, 10);

dt = 1;
velocity_magnitude = 0.5;

a = input('Target X: ');
b = input('Target Y: ');

[a,b] = deg2utm(a,b);

%system initialization.
realSystem = single_agent_rover_system_ROS(LatLon_subscriber, ...
                                                       angle_subscriber, ...
                                                       velocity_publisher, ...
                                                       [a;b], ...
                                                       velocity_magnitude ...
                                                       );    

 %% bounding theta of rover between -pi to pi

 angle_Msg.Data = 90 - angle_Msg.Data;

%% Convert to UTM

[utmX, utmY] = deg2utm(LatLon_Msg.Latitude, LatLon_Msg.Longitude);
%% ........

realSystem.initialCondition = {double([utmX;
                                       utmY;
                                       deg2rad(angle_Msg.Data);
                                       sqrt((a - utmX)^2 + (b - utmY)^2);
                                       (atan2((b - utmY),(a - utmX)))])};
                                                    
mpcOp = ICtMpcOp( ...
                'System'               , realSystem,...
                'HorizonLength'        , 2*dt,...
                'StageCost'            , @(t,x,u,varargin) -x(4));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
                                      'MpcOp'       , dtMpcOp ,...
                                      'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
                                       );

%% VirtualArena Object defined.                                                                      
va = VirtualArena(dtRealSystem,...
                  'StoppingCriteria'  , @(t,sysList)sqrt(((a-dtRealSystem.x(1))^2+(b-dtRealSystem.x(2))^2))<=2 ,...
                  'DiscretizationStep', dt ,...
                  'RealTime'          , 1/dt , ...
                  'PlottingStep'      , 1/dt ...
                 );
log = va.run();