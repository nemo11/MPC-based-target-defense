%PNPP
%moving target

clc; close all; clear all;

attacker_home_subscriber = rossubscriber('/attacker/mavros/home_position/home');
attacker_utm_subscriber = rossubscriber('/attacker/mavros/global_position/local');
attacker_angle_subscriber = rossubscriber('/attacker/mavros/global_position/compass_hdg');
attacker_velocity_publisher = rospublisher('/attacker/mavros/setpoint_velocity/cmd_vel_unstamped');
target_home_subscriber = rossubscriber('/target/mavros/home_position/home');
target_utm_subscriber = rossubscriber('/target/mavros/global_position/local');
target_angle_subscriber = rossubscriber('/target/mavros/global_position/compass_hdg');
target_velocity_publisher = rospublisher('/target/mavros/setpoint_velocity/cmd_vel_unstamped');
defender_home_subscriber = rossubscriber('/defender/mavros/home_position/home');
defender_utm_subscriber = rossubscriber('/defender/mavros/global_position/local');
defender_angle_subscriber = rossubscriber('/defender/mavros/global_position/compass_hdg');
defender_velocity_publisher = rospublisher('/defender/mavros/setpoint_velocity/cmd_vel_unstamped');

dt = 0.1;
vm = 0.4;
vt = 0.2;
vd = 0.5;
K = 10;
N = 10;
switchGuidanceLaw = 10;

attacker_home = receive(attacker_home_subscriber,10);
attacker_utm = receive(attacker_utm_subscriber,10);
attacker_angle = receive(attacker_angle_subscriber,10);
target_home = receive(target_home_subscriber,10);
target_utm = receive(target_utm_subscriber,10);
target_angle = receive(target_angle_subscriber,10);
defender_home = receive(defender_home_subscriber,10);
defender_utm = receive(defender_utm_subscriber,10);
defender_angle = receive(defender_angle_subscriber,10);

%% Convert to UTM
[attacker_home_utmX, attacker_home_utmY] = deg2utm(attacker_home.Latitude, attacker_home.Longitude);
[target_home_utmX, target_home_utmY] = deg2utm(target_home.Latitude, target_home.Longitude);
[defender_home_utmX, defender_home_utmY] = deg2utm(defender_home.Latitude, defender_home.Longitude);

attacker_utmX = attacker_utm.Pose.Pose.Position.X + attacker_home_utmX;
attacker_utmY = attacker_utm.Pose.Pose.Position.Y + attacker_home_utmY;

target_utmX = target_utm.Pose.Pose.Position.X + target_home_utmX;
target_utmY = target_utm.Pose.Pose.Position.Y + target_home_utmY;

defender_utmX = defender_utm.Pose.Pose.Position.X + defender_home_utmX;
defender_utmY = defender_utm.Pose.Pose.Position.Y + defender_home_utmY;

%% Real Vehicle Model
sys = ardurover_MPC_RealVehicleROS1(N, ...
                                   K, ...
                                   switchGuidanceLaw, ...
                                   vm, ...
                                   vd, ...
                                   [attacker_home_utmX, attacker_home_utmY], ...
                                   attacker_utm_subscriber, ...
                                   attacker_angle_subscriber, ...
                                   attacker_velocity_publisher, ...
                                   [target_home_utmX, target_home_utmY], ...
                                   target_utm_subscriber, ...
                                   target_angle_subscriber, ...
                                   target_velocity_publisher, ...
                                   [defender_home_utmX, defender_home_utmY], ...
                                   defender_utm_subscriber, ...
                                   defender_angle_subscriber, ...
                                   defender_velocity_publisher ...
                                   );

%% bounding theta of rover between -pi to pi

 attacker_angle.Data = 90 - attacker_angle.Data;
 defender_angle.Data = 90 - defender_angle.Data;
 target_angle.Data = 90 - target_angle.Data;

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
    'StageConstraints'     , {BoxSet( [-0.8; 0],12:13,[0.8 ; 0.4],12:13,13)},...
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