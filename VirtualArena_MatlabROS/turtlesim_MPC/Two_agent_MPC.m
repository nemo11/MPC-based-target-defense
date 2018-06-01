
clc; close all; clear all;

mysub1 = rossubscriber('/turtle1/pose');
mypub1 = rospublisher('/turtle1/cmd_vel');

mysub2 = rossubscriber('/turtle2/pose');
mypub2 = rospublisher('/turtle2/cmd_vel');

D1 = receive(mysub1 , 10);
D2 = receive(mysub2 , 10);

dt = 1;


%% Unicycle Model

realSystem = RealVehicleROS_V4(mysub1,mypub1,mysub2,mypub2);

theta = D2.Theta;
if( theta > 3.14 )
   theta = theta - 2*3.14;
end
if( theta < -3.14 )
   theta = theta + 2*3.14;
end

realSystem.initialCondition = {double([D2.X;
                                D2.Y;
                                theta;
                                D1.X;
                                D1.Y;
                                sqrt((D1.X - D2.X)^2 + (D1.Y - D2.Y)^2);
                                (atan2((D1.Y - D2.Y),(D1.X - D2.X)))])};
                                                    
mpcOp = ICtMpcOp( ...
    'System'               , realSystem,...
    'HorizonLength'        , 2*dt,...
    'StageCost'            , @(t,x,u,varargin) sqrt((x(1) - x(4))^2 + (x(2) - x(5))^2));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',0) ...
    );

va = VirtualArena(dtRealSystem,...
    'StoppingCriteria'  , @(t,sysList)sqrt(((dtRealSystem.x(4)-dtRealSystem.x(1))^2+(dtRealSystem.x(5)-dtRealSystem.x(2))^2))<0.4 ,...
    'DiscretizationStep', dt ,...
    'RealTime',1/dt , ...
    'PlottingStep'      , 1/dt ,...
    'StepPlotFunction'  , @M2StepPlotFunction ...
    );

log = va.run();
