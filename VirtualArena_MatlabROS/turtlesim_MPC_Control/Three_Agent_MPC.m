%PNPP
%moving target
%EKF
clc; close all; clear all;

attacker_position_subscriber = rossubscriber('/turtle2/pose');
attacker_velocity_publisher = rospublisher('/turtle2/cmd_vel');
target_position_subscriber = rossubscriber('/turtle1/pose');
target_velocity_publisher = rospublisher('/turtle1/cmd_vel');
defender_position_subscriber = rossubscriber('/turtle3/pose');
defender_velocity_publisher = rospublisher('/turtle3/cmd_vel');

dt = 1;
vm = 0.4;
vt = 0.2;
vd = 0.8;
K = 10;
N = 10;
switchGuidanceLaw = 10;

attacker_position = receive(attacker_position_subscriber,10);
target_position = receive(target_position_subscriber,10);
defender_position = receive(defender_position_subscriber,10);

%% Real Vehicle Model
sys = turtlesim_MPC_RealVehicleROS(N, ...
                                   K, ...
                                   switchGuidanceLaw, ...
                                   vm, ...
                                   vt, ...
                                   vd, ...
                                   attacker_position_subscriber, ...
                                   attacker_velocity_publisher, ...
                                   target_position_subscriber, ...
                                   target_velocity_publisher, ...
                                   defender_position_subscriber, ...
                                   defender_velocity_publisher ...
                                   );

if(defender_position.Theta > 3.14)
    defender_position.Theta = defender_position.Theta - 2^3.14;
end

if(defender_position.Theta < -3.14)
    defender_position.Theta = defender_position.Theta + 2^3.14;
end
                               
sys.initialCondition = {double([attacker_position.X;
                                attacker_position.Y;
                                defender_position.X;
                                defender_position.Y;
                                defender_position.Theta;
                                target_position.X;
                                target_position.Y;
                                sqrt((target_position.X - attacker_position.X)^2 + (target_position.Y - attacker_position.Y)^2);
                                atan2(target_position.Y - attacker_position.Y, target_position.X - attacker_position.X);
                                attacker_position.Theta])};

% % System with state and input noise covariance matrices
% Q = diag(([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1])/3)^2;
% R = diag(([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1])/3)^2;
%                      
% for ii = 1:length(sys.initialCondition)
%     x0Filter{ii} = [sys.initialCondition{ii};  %xHat(0)
%                     10*reshape(eye(10),100,1)]; %P(0)
% end
%                      
% sys.stateObserver = EkfFilter(DiscretizedSystem(sys,dt),...
%                  'StateNoiseMatrix'  , dt*Q,...
%                  'OutputNoiseMatrix' , R,...
%                  'InitialCondition'  , x0Filter);


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
    'StoppingCriteria'  , @(t,sysList)sqrt(((dtsys.x(3)-dtsys.x(1))^2+(dtsys.x(4)-dtsys.x(2))^2))<0.5 || sqrt(((dtsys.x(6)-dtsys.x(1))^2+(dtsys.x(7)-dtsys.x(2))^2))<0.5,...
    'DiscretizationStep', dt ,...
    'PlottingStep'      , 1/0.1,...
    'RealTime'          , 1 ...
    );

log = va.run();