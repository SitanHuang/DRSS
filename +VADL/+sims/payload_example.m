%
% This is a simplified demonstration script to simulate the descent of a
% parachute lander after jettisoning from the main rocket.
%

%% Use the states of the main rocket at jettision as initial conditions

uc = DRSS.util.unitConv;

[resultantStates, ~, mainSys] = VADL.sims.solveMain(); % Solve main rocket trajectory first

stateAtJettison = resultantStates.interpolate(mainSys.configParams.jettisonListener.t_trigger - 0.05);
stateAtJettison.t = 0;

%% Construct the lander system

parachute = DRSS.core.dynamics.Parachute() ...
      .setDiameter(4 * 12 * uc.in_to_m) ...
      .setCD(2.2) ...
      .setN(9); % (toroidal = 9, elliptical = 8)

payloadDrag = DRSS.core.dynamics.SimpleDrag() ...
  .setCD(0.5) ...
  .setArea((0.1556/2)^2 * pi) ...
  .setCD_side(0.5) ...
  .setArea_side((0.1556/2)^2 * pi);

sys = DRSS.core.sim.System().overrideM(3.5 * uc.lbm_to_kg) ...
  .setSystemState(stateAtJettison.makeShallowCopy()) ...
  .setLaunchSiteWindSpeed(8 * uc.mph_to_mps) ...
  ...
  .subjectTo(DRSS.core.dynamics.Gravity().setTerminateOnGrounding(true)) ...
  .subjectTo(parachute) ...
  .subjectTo(payloadDrag);

%% Solve

[resultantStates, ~] = DRSS.solver.MatlabODESolver(sys).solve();

%% Plot x-y Trajectory

figure;
plot(resultantStates.x .* uc.m_to_ft, resultantStates.y .* uc.m_to_ft); xlim([000 900]);
hold on;