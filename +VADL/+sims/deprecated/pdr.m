clear; clc; close all;
uc = DRSS.util.unitConv;

%% Set up
mainSys = VADL.config.getMainSys(struct( ...
  'motorLossFactor', (1 - 0.02), ...
  ... 'motorLossFactor', 1, ...
  'disableJettison', false, ...
  'disableDrogue', false, ...
  'windSpeed', 11));

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

fprintf("Avg Thrust-to-Weight: %.1f\n", mean(resultantParameters.params.ThrustToWeight(~isnan(resultantParameters.params.ThrustToWeight))));

ascentStates = resultantStates.interpolate(0:0.005:mainSys.configParams.apogeeListener.t_trigger);
fprintf("Max G (ascent): %.1f\n", max(ascentStates.accelMag) / 9.8);

drogueOpeningStates = resultantStates.interpolate(mainSys.configParams.drogue.t_deploy:0.005:mainSys.configParams.main.t_deploy);
fprintf("Max G (drogue): %.1f\n", max(drogueOpeningStates.accelMag) / 9.8);
mainOpeningStates = resultantStates.interpolate(mainSys.configParams.main.t_deploy:0.005:min(resultantStates.t(end - 10), mainSys.configParams.jettisonListener.t_trigger));
fprintf("Max G (main): %.1f\n", max(mainOpeningStates.accelMag) / 9.8);

heaviestMass = 17.2; % fore

if ~mainSys.configParams.disableJettison
  heaviestMass = heaviestMass - 6.936;
end

foreMass = heaviestMass;

heaviestMass = max(heaviestMass, 11);

fprintf("Landing KE: %.1f lb-ft (65/75)\n", resultantStates.yd(end - 10)^2 * 0.5 * (heaviestMass * uc.lbm_to_kg) * uc.J_to_ftlbf);
fprintf("Landing vel: %.1f fps\n", -resultantStates.yd(end - 10) * uc.mps_to_fps);
fprintf("Descent Time: %.1f s (80/90)\n", resultantStates.t(end) - mainSys.configParams.apogeeListener.t_trigger);
fprintf("Drogue vel: %.1f fps\n", mainSys.configParams.mainDeploymentListener.systemStateAtTrigger.yd * uc.mps_to_fps);

fprintf("Drift: %.0f ft\n", resultantStates.x(end) .* uc.m_to_ft);

apogeeState = mainSys.configParams.apogeeListener.systemStateAtTrigger;

fprintf("Drift from apogee: %.0f ft\n", (apogeeState.x - resultantStates.x(end)) .* uc.m_to_ft)
% return;
%%
[payloadSys, payloadStates, ~] = VADL.sims.payload(mainSys, resultantStates);

payloadStates.t = payloadStates.t + mainSys.configParams.jettisonListener.t_trigger;

return;
%% Plot
% plot(resultantParameters.t, resultantParameters.params.CAr);
% yyaxis right;
plot(resultantParameters.t, resultantParameters.params.CDr);

% plot(resultantParameters.t, resultantParameters.params.ThrustToWeight);
% plot(resultantParameters.t, resultantParameters.params.SSM);


% plot(resultantParameters.t, (resultantParameters.equivForceX.^2 + resultantParameters.equivForceY.^2) .^ (1/2));

% plot(resultantStates.t, resultantStates.yd  .* uc.mps_to_fps);
% plot(resultantStates.t, resultantStates.y  .* uc.m_to_ft);
% plot(resultantStates.t(1:(end - 10)), resultantStates.accelMag(1:(end - 10)) ./ 9.81);
% plot(resultantStates.t, resultantStates.ydd ./ 9.81);

% plot(resultantParameters.t, resultantParameters.m .* uc.kg_to_lbm);

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
saveas(fig, "accel", "png");

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

% plot(payloadStates.t, payloadStates.y .* uc.m_to_ft, 'Color', 'black', 'LineWidth', 2);
% legend('Vehicle', 'Payload')
% saveas(fig, "altitude", "png");

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
saveas(fig, "pitch", "png");

%% Velocity v time
fig = figure;
plot(resultantStates.t, resultantStates.yd .* uc.m_to_ft, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Vertical Velocity");
ylabel("Vertical Velocity [ft/s]")
xlabel("Time [s]");
xlim([resultantStates.t(1) resultantStates.t(end)])
grid on;
hold on;

% plot(payloadStates.t, payloadStates.yd .* uc.m_to_ft, 'Color', 'black', 'LineWidth', 2);
% legend('Vehicle', 'Payload')
% saveas(fig, "velocity", "png");

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

xlim([-2000 3500])
ylim([0 4500])
saveas(fig, "track", "png");

%% Thrust to weight
fig = figure;
plot(resultantParameters.t, resultantParameters.params.ThrustToWeight, 'Color', '#D8AB4C', 'LineWidth', 2);
title("Thrust-to-Weight");
ylabel("Thrust-to-Weight Ratio [1]")
xlabel("Time [s]");
grid on;

% saveas(fig, "thrustratio", "png");

%% SSM v time
meta = mainSys.configParams.rocketDynamics.aerodynamicProfile;

ssm = (-resultantParameters.cgX + meta.CP) / meta.D;

plot(resultantParameters.t, ssm);
xline(mainSys.configParams.launchRail.t_launchRailButtonCleared);
xlim(mainSys.configParams.launchRail.t_launchRailButtonCleared + [-0.05 0.05])
% ssm_LRE = interp1(resultantParameters.t, ssm, mainSys.configParams.launchRail.t_launchRailButtonCleared, 'linear');
% fprintf('SSM at LRE: %.2f\n', ssm_LRE);

%% Sectional Landing KEs

vFinalTerm = resultantStates.yd(end - 10)^2 * 0.5 * uc.J_to_ftlbf;

fprintf("Fore KE: %.1f lb-ft\n", vFinalTerm * foreMass * uc.lbm_to_kg);
fprintf("Mid KE: %.1f lb-ft\n", vFinalTerm * 9.2 * uc.lbm_to_kg);
fprintf("Tail KE: %.1f lb-ft\n", vFinalTerm * 11 * uc.lbm_to_kg);

%% Drift