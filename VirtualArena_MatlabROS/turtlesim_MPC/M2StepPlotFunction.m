function h = M2StepPlotFunction(sysList,log,plot_handles,k)

logX = log{1}.stateTrajectory(:,1:k); hold on;
h    = plot(logX(1,:),logX(2,:),logX(4,:),logX(5,:));
grid on

end
