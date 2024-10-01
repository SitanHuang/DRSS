function [resultantStates, resultantParameters, sys] = solveMain()

uc = DRSS.util.unitConv;

%% Load up VADL configurations
sys = DRSS.core.sim.System("Main Rocket");

VADL.config.gravity(sys);
VADL.config.geometry(sys);
VADL.config.massGroups(sys);

VADL.config.parachutes(sys);
VADL.config.descentDrag(sys);
VADL.config.launchCode(sys);
VADL.config.eventsSetup(sys);
VADL.config.buildFullRocketSystem(sys);

sys.overrideCGX(51.292 * uc.in_to_m);
ssm = (sys.configParams.rocketDynamics.aerodynamicProfile.CP - sys.cgX) / sys.configParams.rocketDynamics.aerodynamicProfile.D


%% Solve
solver = DRSS.solver.MatlabODESolver(sys) ...
  .setCaptureResultantParameters(true);

[resultantStates, resultantParameters] = solver.solve();