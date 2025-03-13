function [sys, resultantStates, resultantParameters] = payload(mainSys, resultantStates)

%% Grab conditions at jettison

% clear;

uc = DRSS.util.unitConv;
%
% [resultantStates, ~, mainSys] = VADL.sims.solveMain();
%
% apogeeState = mainSys.configParams.apogeeListener.systemStateAtTrigger;

jettisonState = resultantStates.interpolate(mainSys.configParams.jettisonListener.t_trigger - 0.1);
jettisonState.t = 0;

% Override as needed:
% jettisonState.xd = -8 / uc.mps_to_fps;

% fprintf("Main descent speed: %.2f fps\n", -jettisonState.yd * uc.mps_to_fps);
% fprintf("Main descent time: %.3f s\n", resultantStates.t(end) - mainSys.configParams.apogeeListener.t_trigger);
% fprintf("Rocket final speed: z=%.2f fps, x=%.2f fps\n", resultantStates.yd(end) * uc.mps_to_fps, resultantStates.xd(end) * uc.mps_to_fps);

%% Construct payload system

sys = DRSS.core.sim.System().overrideM(mainSys.configParams.jettisonEvent.totalJettisonedMass);

% Payload Parachute
parachute = DRSS.core.dynamics.Parachute() ...
      .setDiameter(5 * 12 * uc.in_to_m) ...
      .setCD(2.2) ...
      .setN(9) ... (toroidal = 9, elliptical = 8)
      .setEnabledOnInit(false) ...
      .setDeploymentTimeDelay(-inf);

% Shockcord Travel
shockCordTravel = DRSS.core.dynamics.events.Altitude() ...
  .setAlitude(mainSys.configParams.jettisonAltitude - (25 .* uc.ft_to_m)) ...
  .trigger(parachute);

% Payload Drag

payloadDrag = DRSS.core.dynamics.SimpleDrag() ...
  .setCD(0.5) ...
  .setArea(0.1556 * 0.7 + 4 * 0.254 * 0.03) ...
  .setCD_side(0.5) ...
  .setArea_side(0.1556 * 0.7) ...
  .setEnabledOnInit(false);

sys ...
  .setSystemState(jettisonState.makeShallowCopy()) ...
  .setLaunchSiteElevation(mainSys.launchSiteElevation) ...
  .setLaunchSiteWindSpeed(mainSys.launchSiteWindSpeed) ...
  .setLaunchSiteTemp(mainSys.launchSiteTemp) ...
  .subjectTo( ...
    DRSS.core.dynamics.Gravity() ...
      .setTerminateOnGrounding(true)) ...
  .subjectTo(shockCordTravel) ...
  .subjectTo(parachute) ...
  .subjectTo(payloadDrag);

%% Solve
solver = DRSS.solver.MatlabODESolver(sys) ...
   .setCaptureResultantParameters(true);

[resultantStates, resultantParameters] = solver.solve();

forceZ = gradient(resultantStates.yd(1:(end-5)), resultantStates.t(1:(end-5)));

descentTimeSinceApogee = resultantStates.t(end) - resultantStates.t(1) + mainSys.configParams.jettisonListener.t_trigger - mainSys.configParams.apogeeListener.t_trigger;

% fprintf("Payload max speed: z=%.2f fps\n", max(-resultantStates.yd) * uc.mps_to_fps);
fprintf("Payload final speed: z=%.2f fps, x=%.2f fps\n", resultantStates.yd(end) * uc.mps_to_fps, resultantStates.xd(end) * uc.mps_to_fps);
% fprintf("Payload drift: %.2f ft\n", (resultantStates.x(end) - resultantStates.x(1)) * uc.m_to_ft);
apogeeState = mainSys.configParams.apogeeListener.systemStateAtTrigger;
fprintf("Payload drift from apogee: %.2f ft\n", (resultantStates.x(end) - apogeeState.x) * uc.m_to_ft);
% fprintf("Payload drift from pad: %.2f ft\n", resultantStates.x(end) * uc.m_to_ft);
fprintf("Payload drift (nominal): %.2f ft\n", (descentTimeSinceApogee * mainSys.launchSiteWindSpeed) * uc.m_to_ft);
fprintf("Payload max G: %.2f\n", max(forceZ) / 9.8);
% fprintf("Payload descent time: %.2f s\n", resultantStates.t(end) - resultantStates.t(1));
% fprintf("Payload descent time (+launch): %.2f s\n", resultantStates.t(end) - resultantStates.t(1) + mainSys.configParams.jettisonListener.t_trigger);
fprintf("Payload descent time (+apogee): %.2f s\n", descentTimeSinceApogee);
fprintf("Payload KE: %.3f lb-ft\n", resultantStates.yd(end)^2 * 0.5 * sys.m * uc.J_to_ftlbf);

return;
%% Plot x-y Trajectory

figure;
plot(resultantStates.x .* uc.m_to_ft, resultantStates.y .* uc.m_to_ft); xlim([000 900]);
hold on;
yline(resultantStates.interpolate(parachute.t_complete).y .* uc.m_to_ft, 'Label', 't_{fill}');

%% Plot velocity & forces

figure;
plot(resultantStates.t, resultantStates.yd .* uc.mps_to_fps);
ylabel('Vertical, fps')
xlabel('Time after jettison')
hold on;

yyaxis right;
plot(resultantParameters.t(1:end-5), resultantParameters.equivForceY(1:end-5));
ylabel('Force Y, N')

xline(parachute.t_complete, 'Label', 't_{fill}');
%% Plot acc z
plot(resultantStates.t, gradient(resultantStates.yd, resultantStates.t));
ylabel('Vertical acc, m/s^2')
hold on;
xlim([0 6]);

%% Compare with drop test
dropTestData = readtable("Drop Tests\datalog.csv");

dropTestData.time = dropTestData.time - dropTestData.time(56007);

plot(dropTestData.time(56007:end), dropTestData.accelZ(56007:end));
xlim([0 6]);