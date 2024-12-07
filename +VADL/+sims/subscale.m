%% Solve

uc = DRSS.util.unitConv;

windSpeedMPH = 5;

[resultantStates, resultantParameters, sys] = VADL.sims.solveSubscale(struct( ...
  'windSpeed', windSpeedMPH, ... mph
  'launchAngle', 4 ... deg
));

%% Flight Report
fprintf('Vehicle mass:  %.2f lb\n', sys.m / uc.lbm_to_kg);
fprintf('Vehicle len:   %.2f in\n', sys.len * uc.m_to_in);
fprintf('Vehicle CGx:   %.2f in\n', sys.cgX * uc.m_to_in);
fprintf('Vehicle SSM:   %.2f cal\n', sys.configParams.ssm);

fprintf("SSM: %.2f - %.2f\n", sys.configParams.rocketDynamics.ssm_min, sys.configParams.rocketDynamics.ssm_max);
fprintf("CDr: %.2f - %.2f\n", sys.configParams.rocketDynamics.cdr_min, sys.configParams.rocketDynamics.cdr_max);
fprintf("CAr: %.2f - %.2f\n", max(resultantParameters.params.CAr), min(resultantParameters.params.CAr));
fprintf("CP: %.2f - %.2f\n", sys.configParams.rocketDynamics.cp_calc_min, sys.configParams.rocketDynamics.cp_calc_max);
fprintf("Apogee: %.0f ft\n", max(resultantStates.y) .* uc.m_to_ft);
fprintf("LRE: %.1f fps\n", sys.configParams.launchRail.v_launchRailExit .* uc.mps_to_fps);

fprintf("Avg Thrust-to-Weight: %.1f\n", mean(resultantParameters.params.ThrustToWeight(~isnan(resultantParameters.params.ThrustToWeight))));

fprintf("Landing vel: %.1f fps\n", -resultantStates.yd(end - 10) * uc.mps_to_fps);

descentTime = resultantStates.t(end) - sys.configParams.apogeeListener.t_trigger;

fprintf("Descent Time: %.1f s (80/90)\n", descentTime);

fprintf("Drift: %.0f ft\n", resultantStates.x(end) .* uc.m_to_ft);

apogeeState = sys.configParams.apogeeListener.systemStateAtTrigger;

fprintf("Drift from apogee: %.0f ft\n", (apogeeState.x - resultantStates.x(end)) .* uc.m_to_ft)
fprintf("Nominal drift (descent time * wind): %.0f ft\n", descentTime * windSpeedMPH * 1.46667)

return;
%% Flight track
fig = figure;
plot(resultantStates.x .* uc.m_to_ft, resultantStates.y .* uc.m_to_ft, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Flight Track");
ylabel("Altitude [ft]")
xlabel("Drift [ft]");
grid on;

xlim([-2000 3500]./4)
ylim([0 4500]./3.5)
saveas(fig, "track", "png");

%% Thrust to weight
fig = figure;
plot(resultantParameters.t, resultantParameters.params.ThrustToWeight, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Thrust-to-Weight");
ylabel("Thrust-to-Weight Ratio [1]")
xlabel("Time [s]");
grid on;

saveas(fig, "thrustratio", "png");

%% SSM v time
meta = sys.configParams.rocketDynamics.aerodynamicProfile;

ssm = (-resultantParameters.cgX + meta.CP) / meta.D;

fig = figure;
plot(resultantParameters.t, ssm, 'Color', '#D8AB4C', 'LineWidth', 2);
xline(sys.configParams.launchRail.t_launchRailButtonCleared);
xlim([0.1 sys.configParams.apogeeListener.t_trigger])
title("SSM");
ylabel("SSM Ratio [1]")
xlabel("Time [s]");
saveas(fig, "ssm", "png");
% ssm_LRE = interp1(resultantParameters.t, ssm, sys.configParams.launchRail.t_launchRailButtonCleared, 'linear');
% fprintf('SSM at LRE: %.2f\n', ssm_LRE);