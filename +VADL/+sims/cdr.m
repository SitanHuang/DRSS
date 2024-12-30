clear; clc; close all;
uc = DRSS.util.unitConv;

%% Set up
mainSys = VADL.config.getMainSys(struct( ...
  'disableJettison', false, ...
  'windSpeed', 20));

fprintf('Vehicle mass:  %.2f lb\n', mainSys.m / uc.lbm_to_kg);
fprintf('Vehicle len:   %.2f in\n', mainSys.len * uc.m_to_in);
fprintf('Vehicle CGx:   %.2f in\n', mainSys.cgX * uc.m_to_in);
fprintf('Vehicle SSM:   %.2f cal\n', mainSys.configParams.ssm);

%% Solve

solver = DRSS.solver.MatlabODESolver(mainSys) ...
  .setCaptureResultantParameters(true) ...
  .setPrintPerformanceSummary(true) ...
  ... .configureODE('RelTol', 1e-5, 'AbsTol', 1e-6) ...
  .configureODE('RelTol', 1e-3, 'AbsTol', 1e-4) ...
  .overrideODEFunc(@ode45);

[resultantStates, resultantParameters] = solver.solve();

%% Flight Report
fprintf("SSM: %.2f - %.2f\n", mainSys.configParams.rocketDynamics.ssm_min, mainSys.configParams.rocketDynamics.ssm_max);
fprintf("CDr: %.2f - %.2f\n", mainSys.configParams.rocketDynamics.cdr_min, mainSys.configParams.rocketDynamics.cdr_max);
fprintf("CAr: %.2f - %.2f\n", max(resultantParameters.params.CAr), min(resultantParameters.params.CAr));
fprintf("CP: %.2f - %.2f\n", mainSys.configParams.rocketDynamics.cp_calc_min, mainSys.configParams.rocketDynamics.cp_calc_max);
fprintf("Apogee: %.0f ft\n", max(resultantStates.y) .* uc.m_to_ft);
fprintf("LRE: %.1f fps\n", mainSys.configParams.launchRail.v_launchRailExit .* uc.mps_to_fps);

LREind = find(resultantParameters.t >= mainSys.configParams.launchRail.t_launchRailButtonCleared, 1, 'first');

fprintf("Avg. Thrust-to-Weight: %.1f\n", mean(resultantParameters.params.ThrustToWeight(~isnan(resultantParameters.params.ThrustToWeight))));
fprintf("Thrust-to-Weight at LRE: %.1f\n", resultantParameters.params.ThrustToWeight(LREind));

ascentStates = resultantStates.interpolate(0:0.005:mainSys.configParams.apogeeListener.t_trigger);
fprintf("Max G (ascent): %.1f\n", max(ascentStates.accelMag) / 9.8);

drogueOpeningStates = resultantStates.interpolate(mainSys.configParams.apogeeListener.t_trigger-0.5:0.005:mainSys.configParams.mainDeploymentListener.t_trigger);
fprintf("Max G (drogue): %.1f\n", max(drogueOpeningStates.accelMag) / 9.8);
mainOpeningStates = resultantStates.interpolate(mainSys.configParams.mainDeploymentListener.t_trigger-0.5:0.005:min(resultantStates.t(end - 10), mainSys.configParams.jettisonListener.t_trigger));
fprintf("Max G (main): %.1f\n", max(mainOpeningStates.accelMag) / 9.8);

drogueNames = ["Fore", "Aft"];
drogueMasses = [14.62, 23.27];

drogueKEs = mainOpeningStates.yd(1)^2 * 0.5 * uc.J_to_ftlbf * uc.lbm_to_kg .* drogueMasses;

sectionNames = ["Fore", "Mid", "Aft"];
sectionMasses = [6.56, 13.09, 10.18];

if mainSys.configParams.disableJettison
  sectionMasses = [14.61, 13.09, 10.18];
end

sectionKEs = resultantStates.yd(end - 10)^2 * 0.5 * uc.J_to_ftlbf * uc.lbm_to_kg .* sectionMasses;

descentTime = resultantStates.t(end) - mainSys.configParams.apogeeListener.t_trigger;

fprintf("Drogue KE: %s - %.1f lb-ft (65/75)\n", [drogueNames; drogueKEs]);
fprintf("Landing KE: %s - %.1f lb-ft (65/75)\n", [sectionNames; sectionKEs]);
fprintf("Landing vel: %.1f fps\n", -resultantStates.yd(end - 10) * uc.mps_to_fps);
fprintf("Descent Time: %.1f s (80/90)\n", descentTime);
fprintf("Drogue vel: %.3f fps\n", mainOpeningStates.yd(1) * uc.mps_to_fps);

fprintf("Drift from pad: %.0f ft\n", resultantStates.x(end) .* uc.m_to_ft);

apogeeState = mainSys.configParams.apogeeListener.systemStateAtTrigger;

fprintf("Drift from apogee: %.0f ft\n", (apogeeState.x - resultantStates.x(end)) .* uc.m_to_ft)
fprintf("Drift (nominal): %.0f ft\n", (descentTime .* mainSys.launchSiteWindSpeed) .* uc.m_to_ft)
% return;
%%
[payloadSys, payloadStates, ~] = VADL.sims.payload(mainSys, resultantStates);

payloadStates.t = payloadStates.t + mainSys.configParams.jettisonListener.t_trigger;

return;
%% Plot
% plot(resultantParameters.t, resultantParameters.params.CAr);
% yyaxis right;
% plot(resultantParameters.t, resultantParameters.params.CDr);

% plot(resultantParameters.t, resultantParameters.params.ThrustToWeight);
% plot(resultantParameters.t, resultantParameters.params.SSM);


% plot(resultantParameters.t, (resultantParameters.equivForceX.^2 + resultantParameters.equivForceY.^2) .^ (1/2));

% plot(resultantStates.t, resultantStates.yd  .* uc.mps_to_fps);
% plot(resultantStates.t, resultantStates.y  .* uc.m_to_ft);
% plot(resultantStates.t(1:(end - 10)), resultantStates.accelMag(1:(end - 10)) ./ 9.81);
% plot(resultantStates.t, resultantStates.ydd ./ 9.81);

plot(resultantParameters.t, resultantParameters.m .* uc.kg_to_lbm);

% xlim([mainSys.configParams.main.t_deploy mainSys.configParams.main.t_complete])

% xline(mainSys.configParams.launchRail.t_launchRailButtonCleared);
% xline(mainSys.configParams.drogue.t_deploy);
% xline(mainSys.configParams.drogue.t_complete);
% xline(mainSys.configParams.main.t_deploy);
% xline(mainSys.configParams.main.t_complete);
% xline(mainSys.configParams.jettisonEvent.t_lastEnable);

%% Accel vs time
fig = figure;
plot(resultantStates.t(1:(end - 10)), resultantStates.accelMag(1:(end - 10)) ./ 9.81, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Acceleration");
ylabel("Acceleration, G [1]")
xlabel("Time [s]");
xlim([resultantStates.t(1) resultantStates.t(end - 10)])
grid on;
hold on;

plot(payloadStates.t(10:(end - 10)), payloadStates.accelMag(10:(end - 10)) ./ 9.81, 'Color', 'black', 'LineWidth', 2);
legend('Vehicle', 'Payload');
saveas(fig, "340_accel", "png");
close(fig);

%% Altitude v time
fig = figure;
plot(resultantStates.t, resultantStates.y .* uc.m_to_ft, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Altitude");
ylabel("Altitude [ft]")
xlabel("Time [s]");
xlim([resultantStates.t(1) resultantStates.t(end)])
ylim([0 4500])
grid on;
hold on;

plot(payloadStates.t, payloadStates.y .* uc.m_to_ft, 'Color', 'black', 'LineWidth', 2);
legend('Vehicle', 'Payload')
saveas(fig, "340_altitude", "png");
close(fig);

%% Altitude v time v OpenRocket

openRocketDat = readtable("pdr.openrocket.export.csv.tmp.csv");
plot(openRocketDat.x_Time_s_, openRocketDat.Altitude_ft_, 'Color', '#000000', 'LineWidth', 2);
xlim([resultantStates.t(1) mainSys.configParams.apogeeListener.t_trigger])

legend('VADL-DRSS', 'OpenRocket')

%% Pitch Angle
fig = figure;
plot(resultantStates.t, rad2deg(resultantStates.theta), 'Color', '#D8AB4C', 'LineWidth', 2);
title("Pitch Angle");
ylabel("Pitch Angle [deg]")
xlabel("Time [s]");
xlim([resultantStates.t(1) mainSys.configParams.apogeeListener.t_trigger])
grid on;
saveas(fig, "340_pitch", "png");
close(fig)

%% Velocity v time
fig = figure;
plot(resultantStates.t, resultantStates.yd .* uc.m_to_ft, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Vertical Velocity");
ylabel("Vertical Velocity [ft/s]")
xlabel("Time [s]");
xlim([resultantStates.t(1) resultantStates.t(end)])
grid on;
hold on;

plot(payloadStates.t, payloadStates.yd .* uc.m_to_ft, 'Color', 'black', 'LineWidth', 2);
legend('Vehicle', 'Payload')
saveas(fig, "340_velocity", "png");
close(fig)

%% Velocity v time v OpenRocket

openRocketDat = readtable("pdr.openrocket.export.csv.tmp.csv");
plot(openRocketDat.x_Time_s_, openRocketDat.VerticalVelocity_ft_s_, 'Color', '#000000', 'LineWidth', 2);
xlim([resultantStates.t(1) mainSys.configParams.apogeeListener.t_trigger])
ylim([0 600]);

legend('VADL-DRSS', 'OpenRocket')

%% Flight track
fig = figure;
plot(resultantStates.x .* uc.m_to_ft, resultantStates.y .* uc.m_to_ft, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Flight Track");
ylabel("Altitude [ft]")
xlabel("Drift [ft]");
grid on;
hold on;

plot(payloadStates.x .* uc.m_to_ft, payloadStates.y .* uc.m_to_ft, 'Color', 'black', 'LineWidth', 2);
legend('Vehicle', 'Payload')

ylim([0 4500])
axis equal;
saveas(fig, "340_track", "png");
close(fig)

%% Thrust
fig = figure;
plot(resultantParameters.t, resultantParameters.params.Thrust, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Effective Thrust after Launch Rail Friction Adjustment");
ylabel("Thrust [N]")
xlabel("Time [s]");
grid on;

saveas(fig, "340_thrust", "png");

%% Thrust to weight
fig = figure;
plot(resultantParameters.t, resultantParameters.params.ThrustToWeight, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Thrust-to-Weight after Launch Rail Friction Adjustment");
ylabel("Thrust-to-Weight Ratio [1]")
xlabel("Time [s]");
grid on;

saveas(fig, "340_thrustratio", "png");

%% SSM v time
meta = mainSys.configParams.rocketDynamics.aerodynamicProfile;

ssm = (-resultantParameters.cgX + meta.CP) / meta.D;

plot(resultantParameters.t, ssm);
xline(mainSys.configParams.launchRail.t_launchRailButtonCleared);
xlim(mainSys.configParams.launchRail.t_launchRailButtonCleared + [-0.05 0.05])

%% Sectional Landing KEs

vFinalTerm = resultantStates.yd(end - 10)^2 * 0.5 * uc.J_to_ftlbf;

fprintf("Fore KE: %.1f lb-ft\n", vFinalTerm * foreMass * uc.lbm_to_kg);
fprintf("Mid KE: %.1f lb-ft\n", vFinalTerm * 9.2 * uc.lbm_to_kg);
fprintf("Tail KE: %.1f lb-ft\n", vFinalTerm * 11 * uc.lbm_to_kg);

%% Drift