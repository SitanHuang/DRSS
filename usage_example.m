%% Misc Setup

clear;

addpath(fullfile(fileparts(mfilename('fullpath')), 'VADL'))
addpath(fullfile(fileparts(mfilename('fullpath')), 'VADL/Motor Data'))

uc = DRSS.util.unitConv;

%% Define rocket sections & massing

rocketCylindricalInertialGeometry = struct( ...
  'Ri', (6.125 - 0.25) / 2 * uc.in_to_m, ...
  'Ro', 6.125 / 2 * uc.in_to_m ...
);

noseCone = DRSS.core.obj.Mass("Nose Cone") ...
  .setM(5.1 * uc.lbm_to_kg) ...
  .setLen(8.75 * uc.in_to_m) ...
  .setCGX(10.3300 * uc.in_to_m);
payloadBay = DRSS.core.obj.Mass("Payload bay") ...
  .setM(5.7685 * uc.lbm_to_kg) ...
  .setLen(36 * uc.in_to_m) ...
  .setCGX(22.6789 * uc.in_to_m);
recoveryBay = DRSS.core.obj.Mass("Recovery bay") ...
  .setM(8.5713 * uc.lbm_to_kg) ...
  .setLen(28.5 * uc.in_to_m) ...
  .setCGX(13.586 * uc.in_to_m);
tail = DRSS.core.obj.Mass("Tail") ...
  .setM(2.431 * uc.lbm_to_kg) ...
  .setLen(28 * uc.in_to_m) ...
  .setCGX(9.0820 * uc.in_to_m);

%% Define Rocket Aerodynamics

% TODO:

%% Define Dynamics

% TODO: rocketDyanmics = ...
% TODO: launchRailDynamics = ...
% TODO: apogeeDynamics = ...

% TODO: launchRailDynamics should perform the following for us:
launchRailAngle = deg2rad(5);
launchRailLength = 12 * 12 * uc.in_to_m;

initialState = DRSS.core.sim.SystemState.createZeroState();
initialState.y = launchRailLength .* cos(launchRailAngle) / 2; % Center of launch rail
initialState.theta = launchRailAngle;

gravityDynamics = DRSS.core.dynamics.Gravity() ...
  .setEPS(800 * uc.ft_to_m) ... % set launch site elevation above mean sea level
  .setTerminateOnGrounding(true) ...
  .setGroundingHeight(initialState.y) ...
  .setGroundingTimeThreshold(1); % do not terminate mission even if rocket touches ground during the first second

motorDynamics = DRSS.core.dynamics.Motor( ...
  fullfile(fileparts(mfilename('fullpath')), 'VADL/Motor Data/L1400.csv'), ...
  "L1400", @drss_motor_database);
motor = motorDynamics.genMotorMass(); % the motor Mass is automatically bound to this motorDynamics

%% Simulate Rocket Ascent

% System must be the top level MassGroup to calculate
% correctly the sectional moments of inertia; this is
% because non-System MassGroups calculate moments of
% inertia by assuming homogenous distribution in length
sys = DRSS.core.sim.System("Ascent to Apogee") ...
  .appendChild(noseCone) ...
  .appendChild(payloadBay) ...
  .appendChild(recoveryBay) ...
  .appendChild(tail) ...
  .appendChild(motor) ...
  .setInertialGeometryRecursive(rocketCylindricalInertialGeometry) ...
  .subjecTo(gravityDynamics) ...
  .subjecTo(motorDynamics) ...
  .setSystemState(initialState);

% Hyperparameter optimization for ODE45Solver is extremely important; see the
% ODE45Solver.m source code for more detailed information on how to optimize
% both accuracy and computation time by carefully modyfing solver.ODEOptions
solver = DRSS.solver.ODE45Solver(sys) ...
  .setCaptureResultantParameters(true) ... % Whether to capture time variant params (i.e., m, mdot, I), which slows down the solver drastically
  .setPrintPerformanceSummary(true);

% solver.debugFlag = true;

[resultantStates, resultantParameters] = solver.solve();

%% WIP; Debug & dev only:

% plot(resultantStates.t, gradient(resultantStates.yd, resultantStates.t))
% plot(resultantStates.t, resultantStates.y .* uc.m_to_ft)
% plot(resultantStates.x .* uc.m_to_ft, resultantStates.y .* uc.m_to_ft)
% plot(resultantStates.t, rad2deg(resultantStates.theta))
plot(resultantParameters.t, resultantParameters.m .* uc.kg_to_lbm)
% plot(resultantParameters.t, resultantParameters.I)
% plot(resultantParameters.t, resultantParameters.equivForceY)
