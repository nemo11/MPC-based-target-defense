%PNPP
%moving target
%EKF
clc; close all; clear all;

dt = 0.01;
vm=40;
vt=20;
vd=40;
K=3;
N=3;
switchGuidanceLaw = 10;
%% Unicycle Model
sys = ICtSystem(...
      'StateEquation', @(t,x,u,varargin) [
    vm*cos(x(9));
    vm*sin(x(9));
    vd*cos(u(1));
    vd*sin(u(1));
    vt*cos(u(2));
    vt*sin(u(2));
    vt*cos(u(2)-x(8))-vm*cos(x(9)-x(8));
    (vt*sin(u(2)-x(8))-vm*sin(x(9)-x(8)))/x(7);
    ((((-1).^floor(ceil(t/switchGuidanceLaw)))+1)/2)*(N*((vt*sin(u(2)-x(8))-vm*sin(x(9)-x(8)))/x(7)))+(1-((((-1).^floor(ceil(t/switchGuidanceLaw)))+1)/2))*(((-K*(x(9)-x(8)))/vm))],...
    'OutputEquation', @(t,x,varargin) x(1:9), 'ny', 9,...
    'nx',9,'nu',2 ...
);

a1=0;
a2=0;
d1=150;
d2=200;
t1=50;
t2=70;
alm=0;
R1=sqrt((t1-a1)^2+(t2-a2)^2);
Th=atan2((t2-a2),(t1-a1));

% System with state and input noise covariance matrices
Q = diag(([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1])/3)^2;
R = diag(([0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1])/3)^2;

realSystem = ICtSystem(...
            'StateEquation',  @(t,x,u,varargin) sys.f(t,x,u,varargin{:}) + chol(Q)*randn(9,1),...
            'OutputEquation', @(t,x,varargin)   sys.h(t,x,varargin{:})   + chol(R)*randn(9,1), 'ny', 9,...
            'nx',9,'nu',2 ...
            );

realSystem.initialCondition = {[a1;a2;d1;d2;t1;t2;R1;Th;alm]};

for ii = 1:length(realSystem.initialCondition)
    x0Filter{ii} = [realSystem.initialCondition{ii} + 5*randn(9,1);  %xHat(0)
                    10*reshape(eye(9),81,1)                          ]; %P(0)
end

realSystem.stateObserver = EkfFilter(DiscretizedSystem(sys,dt),...
                 'StateNoiseMatrix'  , dt*Q,...
                 'OutputNoiseMatrix' , R,...
                 'InitialCondition'  , x0Filter);


mpcOp = ICtMpcOp( ...
    'System'               , realSystem,...
    'HorizonLength'        , 2*dt,...
    'StageCost'            , @(t,x,u,varargin)-((x(5)-x(1))^2+(x(6)-x(2))^2)+((x(3)-x(1))^2+(x(4)-x(2))^2));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
    );

va = VirtualArena(dtRealSystem,...
    'StoppingCriteria'  , @(t,sysList)sqrt(((dtRealSystem.x(3)-dtRealSystem.x(1))^2+(dtRealSystem.x(4)-dtRealSystem.x(2))^2))<0.2 || sqrt(((dtRealSystem.x(5)-dtRealSystem.x(1))^2+(dtRealSystem.x(6)-dtRealSystem.x(2))^2))<0.2,...
    'PlottingStep'      , 1/0.1, ...
    'StepPlotFunction'  , @threeagent_ekf2_plot_function ...
    );

log = va.run();

