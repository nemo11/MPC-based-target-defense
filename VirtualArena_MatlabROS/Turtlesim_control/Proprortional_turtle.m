clc; close all;
mysub = rossubscriber('/turtle1/pose');
mypub = rospublisher('/turtle1/cmd_vel');

dt = 0.1;
%kp = 1; % kp value is declared inside the controller function.

recvMsg = receive(mysub , 10);

a = input('Target X: ');
b = input('Target Y: ');
target = [a , b];

sys = RealVehicleROS_V3(mysub , mypub , target);
%sys = T_Model(mysub , mypub);

sys.initialCondition = {[recvMsg.X;recvMsg.Y;recvMsg.Theta]};

sys.controller = Modified_IController(@(t,x) (atan2((b - x(2)),(a - x(1))) - x(3)));

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-a)*(sys.x(1)-a) + (sys.x(2)-b)*(sys.x(2)-b))<=0.1,...
    'DiscretizationStep', dt ,...
    'RealTime',1 , ...
    'PlottingStep'      , 1 ,...
    'StepPlotFunction'  , @ex01StepPlotFunction);
  
log = va.run();