%% Solve

uc = DRSS.util.unitConv;

[resultantStates, resultantParameters, sys] = VADL.sims.solveMain();

%% Flight Report
fprintf("SSM: %.2f - %.2f\n", sys.configParams.rocketDynamics.ssm_min, sys.configParams.rocketDynamics.ssm_max);
fprintf("Apogee: %.3f ft\n", max(resultantStates.y) .* uc.m_to_ft);
fprintf("LRE: %.3f fps\n", sys.configParams.launchRail.v_launchRailExit .* uc.mps_to_fps);
fprintf("Drift: %.3f ft\n", resultantStates.x(end) .* uc.m_to_ft);
fprintf("Landing KE: %.3f lb-ft\n", resultantStates.yd(end)^2 * 0.5 * sys.configParams.recoveryBay.m * uc.J_to_ftlbf);
fprintf("Landing vel: %.3f fps\n", -resultantStates.yd(end) * uc.mps_to_fps);
fprintf("Descent Time: %.3f s\n", resultantStates.t(end) - sys.configParams.apogeeListener.t_trigger);

%% Plot x-y Trajectory

plot(resultantStates.x .* uc.m_to_ft, resultantStates.y .* uc.m_to_ft); xlim([-500 7000]);

%% Extract Jettison SystemState
resultantStates.interpolate(sys.configParams.jettisonListener.t_trigger)

%% Linearize
linearizedState = resultantStates.interpolate(1:0.05:resultantStates.t(end));

%% Plot theta over time

plot(resultantStates.t, rad2deg(resultantStates.theta))
xline(sys.configParams.launchRail.t_launchRailCGCleared);
xline(sys.configParams.apogeeListener.t_trigger);
xlim([0, sys.configParams.apogeeListener.t_trigger + 2]);