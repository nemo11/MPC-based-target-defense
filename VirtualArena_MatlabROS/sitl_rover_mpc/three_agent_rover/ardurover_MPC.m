%PNPP
%moving target

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

dt = 0.95;
vm = 0.9;
vt = 0.3;
vd = 0.6;
K = 10;
N = 3;
switchGuidanceLaw = 10;

attacker_LatLon = receive(attacker_LatLon_subscriber,10);
attacker_angle = receive(attacker_angle_subscriber,10);
target_LatLon = receive(target_LatLon_subscriber,10);
target_angle = receive(target_angle_subscriber,10);
defender_LatLon = receive(defender_LatLon_subscriber,10);
defender_angle = receive(defender_angle_subscriber,10);

%% Real Vehicle Model
sys = ardurover_MPC_RealVehicleROS(N, ...
                                   K, ...
                                   switchGuidanceLaw, ...
                                   vm, ...
                                   vt, ...
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

%% bounding theta of rover between -pi to pi

 attacker_angle.Data = 90 - attacker_angle.Data;
 defender_angle.Data = 90 - defender_angle.Data;

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
                                sqrt((target_utmX - attacker_utmX)^2 + (target_utmY - attacker_utmY)^2);
                                atan2(target_utmY - attacker_utmY, target_utmX - attacker_utmX);
                                deg2rad(attacker_angle.Data)])};

mpcOp = ICtMpcOp( ...
    'System'               , sys,...
    'HorizonLength'        , 2*dt,...
    'StageCost'            , @(t,x,u,varargin) -((x(6)-x(1))^2+(x(7)-x(2))^2)+((x(3)-x(1))^2+(x(4)-x(2))^2));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtsys = DiscretizedSystem(sys,dt);

dtsys.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp, 'UseSymbolicEvaluation', 1) ...
    );

va = VirtualArena(dtsys,...
    'StoppingCriteria'  , @(t,sysList)sqrt(((dtsys.x(3)-dtsys.x(1))^2+(dtsys.x(4)-dtsys.x(2))^2))<1 || sqrt(((dtsys.x(6)-dtsys.x(1))^2+(dtsys.x(7)-dtsys.x(2))^2))<1,...
    'DiscretizationStep', dt ,...
    'PlottingStep'      , 1/0.1,...
    'RealTime'          , 1 ...
    );

log = va.run();