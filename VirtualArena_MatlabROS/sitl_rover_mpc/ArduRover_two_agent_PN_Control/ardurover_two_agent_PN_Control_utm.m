clc; clear all; close all;

attacker_LatLon_subscriber = rossubscriber('/attacker/mavros/global_position/global');
attacker_angle_subscriber = rossubscriber('/attacker/mavros/global_position/compass_hdg');
attacker_velocity_publisher = rospublisher('/attacker/mavros/setpoint_velocity/cmd_vel_unstamped');
target_LatLon_subscriber = rossubscriber('/target/mavros/global_position/global');
target_angle_subscriber = rossubscriber('/target/mavros/global_position/compass_hdg');
target_velocity_publisher = rospublisher('/target/mavros/setpoint_velocity/cmd_vel_unstamped');

dt = 1;
vm = 0.4;
vt = 0.2;
N = 10; 

attacker_LatLon = receive(attacker_LatLon_subscriber,10);
attacker_angle = receive(attacker_angle_subscriber,10);
target_LatLon = receive(target_LatLon_subscriber,10);
target_angle = receive(target_angle_subscriber,10);

%% bounding theta of rover between -pi to pi

 attacker_angle.Data = 90 - attacker_angle.Data;
 target_angle.Data = 90 - target_angle.Data;

%% Convert to UTM

[attacker_utmX, attacker_utmY] = deg2utm(attacker_LatLon.Latitude, attacker_LatLon.Longitude);
[target_utmX, target_utmY] = deg2utm(target_LatLon.Latitude, target_LatLon.Longitude);

sys = ardurover_two_agent_PN_RealVehicleROS_utm(vm, ...
                                   vt, ...
                                   attacker_LatLon_subscriber, ...
                                   attacker_angle_subscriber, ...
                                   attacker_velocity_publisher, ...
                                   target_LatLon_subscriber, ...
                                   target_angle_subscriber, ...
                                   target_velocity_publisher ...
                                   );

sys.initialCondition = {[attacker_utmX;
                        attacker_utmY;
                        target_utmX;
                        target_utmY;
                        sqrt((target_utmX - attacker_utmX)^2 + (target_utmY - attacker_utmY)^2);
                        atan2(target_utmY - attacker_utmY, target_utmX - attacker_utmX);
                        deg2rad(attacker_angle.Data);
                        deg2rad(target_angle.Data)]};

sys.controller = ardurover_two_agent_PN_IController_utm(@(t,x) ((vt*sin(x(8)-x(6))-vm*sin(x(7)-x(6)))/x(5)),N);
% sys.controller = ardurover_two_agent_PN_IController_utm(@(t,x) (x(6) - x(7)),N);

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1) - sys.x(3))^2 + (sys.x(2) - sys.x(4))^2)<=0.5,...
    'DiscretizationStep', dt ,...
    'RealTime'          , 1);
%     'PlottingStep'      , 1 ,...
%     'StepPlotFunction'  , @ex01StepPlotFunction);
  
log = va.run();