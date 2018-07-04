%Target starts moving after the attacker has reached closer
%Target and Defender uses Model Predictive Control
%Attacker uses Proportional Navigation

clc; close all; clear all;

attacker_LatLon_subscriber = rossubscriber('/attacker/mavros/global_position/global');
attacker_angle_subscriber = rossubscriber('/attacker/mavros/global_position/compass_hdg');
attacker_velocity_publisher = rospublisher('/attacker/mavros/setpoint_velocity/cmd_vel_unstamped');
target_LatLon_subscriber = rossubscriber('/target/mavros/global_position/global');
target_angle_subscriber = rossubscriber('/target/mavros/global_position/compass_hdg');
target_velocity_publisher = rospublisher('/target/mavros/setpoint_velocity/cmd_vel_unstamped');
defender_LatLon_subscriber = rossubscriber('/defender/mavros/global_position/global');
defender_angle_subscriber = rossubscriber('/defender/mavros/global_position/compass_hdg');
defender_velocity_publisher = rospublisher('/defender/mavros/setpoint_velocity/cmd_vel_unstamped');

dt = 0.1;
vm = 0.5;                   %attacker linear velocity
max_vt = 0.2;               %maximum target linear velocity
vd = 1;                     %defender linear velocity
e = 5;                      %safety distance between the attacker and target
K = 10;                     
N = 15;                     %proportional constant of Proportional Navigation
switchGuidanceLaw = 10;

attacker_LatLon = receive(attacker_LatLon_subscriber,10);
attacker_angle = receive(attacker_angle_subscriber,10);
target_LatLon = receive(target_LatLon_subscriber,10);
target_angle = receive(target_angle_subscriber,10);
defender_LatLon = receive(defender_LatLon_subscriber,10);
defender_angle = receive(defender_angle_subscriber,10);

%% Real Vehicle Model(the state space equations are defined inside the function)
sys = case2_ardurover_MPC_RealVehicleROS1(N, ...                                 
                                   K, ...
                                   switchGuidanceLaw, ...
                                   vm, ...
                                   vd, ...
                                   attacker_LatLon_subscriber, ...
                                   attacker_angle_subscriber, ...
                                   attacker_velocity_publisher, ...
                                   target_LatLon_subscriber, ...
                                   target_angle_subscriber, ...
                                   target_velocity_publisher, ...
                                   defender_LatLon_subscriber, ...
                                   defender_angle_subscriber, ...
                                   defender_velocity_publisher ...
                                   );

%% angle correction for conversion from lat-long to utm co-ordinate system
 attacker_angle.Data = 90 - attacker_angle.Data;
 defender_angle.Data = 90 - defender_angle.Data;
 target_angle.Data = 90 - target_angle.Data;

%% Convert to UTM
[attacker_utmX, attacker_utmY] = deg2utm(attacker_LatLon.Latitude, attacker_LatLon.Longitude);
[target_utmX, target_utmY] = deg2utm(target_LatLon.Latitude, target_LatLon.Longitude);
[defender_utmX, defender_utmY] = deg2utm(defender_LatLon.Latitude, defender_LatLon.Longitude);

%% ......                               
sys.initialCondition = {double([attacker_utmX;
                                attacker_utmY;
                                defender_utmX;
                                defender_utmY;
                                deg2rad(defender_angle.Data);
                                target_utmX;
                                target_utmY;
                                deg2rad(target_angle.Data);
                                sqrt((target_utmX - attacker_utmX)^2 + (target_utmY - attacker_utmY)^2);
                                atan2(target_utmY - attacker_utmY, target_utmX - attacker_utmX);
                                deg2rad(attacker_angle.Data)])};
                            
mpcOp = ICtMpcOp( ...
    'System'               , sys,...
    'HorizonLength'        , 2*dt,...
    'StageConstraints'     , {BoxSet( [-0.8; 0 ; -1],12:14,[0.8 ; max_vt ; 1],12:14,14)},...
    'StageCost'            , @(t,x,u,varargin) (1-not(not(floor(e/(sqrt((x(6)-x(1))^2+(x(7)-x(2))^2))))))*(u(2)^2 + u(3)^2)...
                            -(not(not(floor(e/(sqrt((x(6)-x(1))^2+(x(7)-x(2))^2)))))*((x(6)-x(1))^2+(x(7)-x(2))^2))...
                            +((x(3)-x(1))^2+(x(4)-x(2))^2));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtsys = DiscretizedSystem(sys,dt);

dtsys.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp, 'UseSymbolicEvaluation', 1) ...
    );

va = VirtualArena(dtsys,...
    'StoppingCriteria'  , @(t,sysList)sqrt(((dtsys.x(3)-dtsys.x(1))^2+(dtsys.x(4)-dtsys.x(2))^2))<1 || sqrt(((dtsys.x(6)-dtsys.x(1))^2+(dtsys.x(7)-dtsys.x(2))^2))<1,...
    'DiscretizationStep', dt ,...
    'RealTime'          , 1 ...
    );

log = va.run();