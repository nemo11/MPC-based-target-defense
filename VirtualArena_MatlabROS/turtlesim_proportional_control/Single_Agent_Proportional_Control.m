% This code is for moving a single agent (i.e, a turtle) from the initial
% position to a given target. The turtle follows proportional controller to
% achieve its goal.

clc; close all;
agent_pose_subscriber = rossubscriber('/turtle1/pose');
agent_velocity_publisher = rospublisher('/turtle1/cmd_vel');

dt = 0.1;
kp = 2;
velocity = 1;
agent_pose_data = receive(agent_pose_subscriber , 10);

a = input('Target X: ');
b = input('Target Y: ');
target = [a , b];

sys = Single_Agent_RealVehicleROS(agent_pose_subscriber , agent_velocity_publisher , target , velocity);

sys.initialCondition = {[agent_pose_data.X;agent_pose_data.Y;agent_pose_data.Theta]};

% The controller equation the given as an input in this function.
sys.controller = Modified_IController(@(t,x) (atan2((target(2) - x(2)),(target(1) - x(1))) - x(3)) , kp);

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-a)*(sys.x(1)-a) + (sys.x(2)-b)*(sys.x(2)-b))<=0.1,...
    'DiscretizationStep', dt ,...
    'RealTime',1 , ...
    'PlottingStep'      , 1 ,...
    'StepPlotFunction'  , @ex01StepPlotFunction);
  
log = va.run();