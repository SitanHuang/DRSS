function [resultantStates, resultantParameters, sys] = solveSubscale(configParams)

uc = DRSS.util.unitConv;

%% Load up VADL configurations
sys = VADL.config.subscale.getSubscaleSys(configParams);


%% Solve
solver = DRSS.solver.MatlabODESolver(sys) ...
  .setCaptureResultantParameters(true) ...
  .configureODE('RelTol', 1e-9, 'AbsTol', 1e-10) ...
  .overrideODEFunc(@ode113);

[resultantStates, resultantParameters] = solver.solve();