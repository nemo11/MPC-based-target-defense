%% Using MPC controller to decrease the distance between two turtle.
%% Turtle1 moving with constant velocity, Turtle2 using control variable from MPC to minimize the given cost function.

clc; close all; clear all;

% publisher,subscriber and other required initializations....

turtle1_Pose_Subscriber = rossubscriber('/turtle1/pose');
turtle1_Vel_Publisher = rospublisher('/turtle1/cmd_vel');

turtle2_Pose_Subscriber = rossubscriber('/turtle2/pose');
turtle2_Vel_Publisher = rospublisher('/turtle2/cmd_vel');

turtle1_Pose_Data = receive(turtle1_Pose_Subscriber , 10);
turtle2_Pose_Data = receive(turtle2_Pose_Subscriber , 10);

dt = 1;

realSystem = RealVehicleROS_V4(turtle1_Pose_Subscriber,turtle1_Vel_Publisher,turtle2_Pose_Subscriber,turtle2_Vel_Publisher);    %system initialization.

%% Angle correction for turtlesim in order to constain it between -pi to pi 
theta = turtle2_Pose_Data.Theta;

if( theta > 3.14 )
   theta = theta - 2*3.14;
end
if( theta < -3.14 )
   theta = theta + 2*3.14;
end
%% ........

realSystem.initialCondition = {double([turtle2_Pose_Data.X;
                                turtle2_Pose_Data.Y;
                                theta;
                                turtle1_Pose_Data.X;
                                turtle1_Pose_Data.Y;
                                sqrt((turtle1_Pose_Data.X - turtle2_Pose_Data.X)^2 + (turtle1_Pose_Data.Y - turtle2_Pose_Data.Y)^2);
                                (atan2((turtle1_Pose_Data.Y - turtle2_Pose_Data.Y),(turtle1_Pose_Data.X - turtle2_Pose_Data.X)))])};
                                                    
mpcOp = ICtMpcOp( ...
                'System'               , realSystem,...
                'HorizonLength'        , 2*dt,...
                'StageCost'            , @(t,x,u,varargin) -x(6));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
                                      'MpcOp'       , dtMpcOp ,...
                                      'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
                                       );

%% VirtualArena Object defined.                                   
                                   
va = VirtualArena(dtRealSystem,...
                  'StoppingCriteria'  , @(t,sysList)sqrt(((dtRealSystem.x(4)-dtRealSystem.x(1))^2+(dtRealSystem.x(5)-dtRealSystem.x(2))^2))<0.4 ,...
                  'DiscretizationStep', dt ,...
                  'RealTime',1/dt , ...
                  'PlottingStep'      , 1/dt ,...
                  'StepPlotFunction'  , @M2StepPlotFunction ...
                 );
             
log = va.run();
