%% Grab conditions at jettison

uc = DRSS.util.unitConv;

[resultantStates, ~, mainSys] = VADL.sims.solveMain();

jettisonState = resultantStates.interpolate(mainSys.configParams.jettisonListener.t_trigger - 0.05);
jettisonState.t = 0;

% Override as needed:
jettisonState.xd = -8 / uc.mps_to_fps;

% Legacy: 18.45, DRSS: 18.17
fprintf("Main descent speed: %.2f fps\n", -jettisonState.yd * uc.mps_to_fps);
fprintf("Rocket final speed: z=%.2f fps, x=%.2f fps\n", resultantStates.yd(end) * uc.mps_to_fps, resultantStates.xd(end) * uc.mps_to_fps);

%% Construct payload system

sys = DRSS.core.sim.System().overrideM(5.1 * uc.lbm_to_kg);

% Payload Parachute
parachute = DRSS.core.dynamics.Parachute() ...
      .setDiameter(4 * 12 * uc.in_to_m) ...
      .setCD(2.2) ...
      .setN(9) ... (toroidal = 9, elliptical = 8)
      .setEnabledOnInit(true);

% Payload Drag

payloadDrag = DRSS.core.dynamics.SimpleDrag() ...
  .setCD(0.5) ...
  .setArea((0.1556/2)^2 * pi) ...
  .setCD_side(0.5) ...
  .setArea_side((0.1556/2)^2 * pi);

sys ...
  .setSystemState(jettisonState.makeShallowCopy()) ...
  .setLaunchSiteElevation(800 * uc.ft_to_m) ...
  .setLaunchSiteWindSpeed(20 * uc.mph_to_mps) ...
  .subjectTo( ...
    DRSS.core.dynamics.Gravity() ...
      .setTerminateOnGrounding(true)) ...
  .subjectTo(parachute) ...
  .subjectTo(payloadDrag);

%% Solve
solver = DRSS.solver.MatlabODESolver(sys) ...
   .setCaptureResultantParameters(true);

[resultantStates, resultantParameters] = solver.solve();

fprintf("Payload final speed: z=%.2f fps, x=%.2f fps\n", resultantStates.yd(end) * uc.mps_to_fps, resultantStates.xd(end) * uc.mps_to_fps);
fprintf("Payload drift: %.2f ft\n", (resultantStates.x(end) - resultantStates.x(1)) * uc.m_to_ft);

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