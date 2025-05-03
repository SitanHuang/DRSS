function [resultantStates, resultantParameters, sys] = solveMain(configParams)

if nargin < 1
  configParams = struct();
end

%% Load up VADL configurations
sys = VADL.config.getMainSys(configParams);

%% Solve
solver = DRSS.solver.MatlabODESolver(sys) ...
  .setCaptureResultantParameters(true) ...
  .setPrintPerformanceSummary(true) .....
  .configureODE('RelTol', 1e-12, 'MaxStep', 0.05) ...
  .overrideODEFunc(@ode113);

[resultantStates, resultantParameters] = solver.solve();