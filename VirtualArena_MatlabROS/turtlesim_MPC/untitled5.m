%PNPP
%moving target
%EKF
clc; close all; clear all;

mysub1 = rossubscriber('/turtle1/pose');
mypub1 = rospublisher('/turtle1/cmd_vel');

mysub2 = rossubscriber('/turtle2/pose');
mypub2 = rospublisher('/turtle2/cmd_vel');

D1 = receive(mysub1 , 10);
D2 = receive(mysub2 , 10);

dt = 0.1;


%% Unicycle Model

realSystem = RealVehicleROS_V4(mysub1,mypub1,mysub2,mypub2);

realSystem.initialCondition = {[D1.X;
                                D1.Y;
                                D2.X;
                                D2.Y;
                                ((D1.X - D2.X)^2 + (D1.Y - D2.Y)^2);
                                (atan2((D1.Y - D2.Y),(D1.X - D2.X)))]};

mpcOp = ICtMpcOp( ...
    'System'               , realSystem,...
    'HorizonLength'        , 1.2*dt,...
    'StageCost'            , @(t,x,u,varargin) -((x(3)-x(1))^2+(x(4)-x(2))^2));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp) ...
    );

va = VirtualArena(dtRealSystem,...
    'StoppingCriteria'  , @(t,sysList)sqrt(((dtRealSystem.x(3)-dtRealSystem.x(1))^2+(dtRealSystem.x(4)-dtRealSystem.x(2))^2))<0.4 ,...
    'PlottingStep'      , 1/0.1, ...
    'StepPlotFunction'  , @M2StepPlotFunction ...
    );

log = va.run();
