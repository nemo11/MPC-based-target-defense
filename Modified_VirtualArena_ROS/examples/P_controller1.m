clc; close all; clear all;

 sys = CtSystem('StateEquation',@(t,x,u) [
    cos(x(3))*u(1);
    sin(x(3))*u(1);
    u(2)],'nx',3, 'nu',2);

 sys.initialCondition = [0;0;pi/2];

 points = [ 0,0; 0,15; 5,15; 5,5; 15,5; 15,2; %L
            20,2;  30,15; 40,0; 35,0; 33,4; 27,4; 20,0]';

 plot(points(1,:),points(2,:),'o'); hold on


sys.controller = GoToWayPoints(points,'DetectionRadius',1, 'InputBoxSetConstraints',BoxSet([0;-pi/3],[2;pi/3]));

va = VirtualArena(sys,...
    'StoppingCriteria',@(t,sysList)t>80,...
    'StepPlotFunction', @(systemsList,log,oldHandles,k) plot(log{1}.stateTrajectory(1,1:k),log{1}.stateTrajectory(2,1:k)),... @stepPoltTrajecotry, ...
    'DiscretizationStep',0.1);

ret = va.run();