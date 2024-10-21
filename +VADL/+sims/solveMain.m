function [resultantStates, resultantParameters, sys] = solveMain()

uc = DRSS.util.unitConv;

%% Load up VADL configurations
sys = VADL.config.getMainSys();

% sys.overrideCGX(51.292 * uc.in_to_m);
% ssm = (sys.configParams.rocketDynamics.aerodynamicProfile.CP - sys.cgX) / sys.configParams.rocketDynamics.aerodynamicProfile.D


%% Solve
solver = DRSS.solver.MatlabODESolver(sys) ...
  .setCaptureResultantParameters(true);

[resultantStates, resultantParameters] = solver.solve();