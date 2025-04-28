clear; clc; close all;
uc = DRSS.util.unitConv;

%% Set up

% Lebanon, TN:
mainSys = VADL.config.getMainSys(struct( ...
  'disableJettison', false, ...
  'noMotor', false, ...
  'motorLossFactor', 0.965, ...
  'noBallast', false, ...
  'launchAngle', 7, ...
  'launchSiteElevation', 800, ...
  'launchSiteTemp', 50, ...
  'launchSitePressurePa', 91550, ...
  'motorOverride', 'L1720', ...
  'launchRailEffectiveTravel', 12, ... % ft
  'windSpeed', [[0, 11]; [1020, 12]; [4075, 11]; [5561, 13]], ...
  'driftCalcWindSpeed', 10, ...
  'windModelLowSpeed', 10, ...
  'windModelHighSpeed', 19, ...
  'windModelTurbulence', 0.08, ...
  'windModelFreq', 1 ...
));

fprintf('Vehicle mass:  %.2f lb\n', mainSys.m / uc.lbm_to_kg);
if ~(isfield(mainSys.configParams, 'noMotor') && mainSys.configParams.noMotor)
  motor_param = mainSys.configParams.motorDynamics.motor_params;
  fprintf('       (dry):  %.2f lb\n', (mainSys.m - motor_param.m_prop0) / uc.lbm_to_kg);
  fprintf('  (no motor):  %.2f lb\n', (mainSys.m - motor_param.m_prop0 - motor_param.m0) / uc.lbm_to_kg);
end
fprintf('Vehicle len:   %.2f in\n', mainSys.len * uc.m_to_in);
fprintf('Vehicle CGx:   %.2f in\n', mainSys.cgX * uc.m_to_in);
fprintf('Vehicle SSM:   %.2f cal\n', mainSys.configParams.ssm);

if isfield(mainSys.configParams, 'noMotor') && mainSys.configParams.noMotor
  return;
end

%% Solve

solver = DRSS.solver.MatlabODESolver(mainSys) ...
  .setCaptureResultantParameters(true) ...
  .setPrintPerformanceSummary(true) .....
  .configureODE('RelTol', 1e-12, 'MaxStep', 0.05) ...
  .overrideODEFunc(@ode113);

[resultantStates, resultantParameters] = solver.solve();

% Flight Report
fprintf("SSM: %.2f - %.2f\n", mainSys.configParams.rocketDynamics.ssm_min, mainSys.configParams.rocketDynamics.ssm_max);
fprintf("CDr: %.2f - %.2f\n", mainSys.configParams.rocketDynamics.cdr_min, mainSys.configParams.rocketDynamics.cdr_max);
fprintf("CAr: %.2f - %.2f\n", max(resultantParameters.params.CAr), min(resultantParameters.params.CAr));
fprintf("CP: %.2f - %.2f, %.2f\n", mainSys.configParams.rocketDynamics.cp_calc_min .* uc.m_to_in, mainSys.configParams.rocketDynamics.cp_calc_max .* uc.m_to_in, mainSys.configParams.rocketDynamics.cp_calc_0 .* uc.m_to_in);
fprintf("Apogee: %.0f ft\n", max(resultantStates.y) .* uc.m_to_ft);

% LREind = find(resultantStates.t > mainSys.configParams.launchRail.t_launchRailButtonCleared, 1, 'first');

fprintf("LRE: %.1f fps\n", mainSys.configParams.launchRail.v_launchRailExit .* uc.mps_to_fps);

LREind = find(resultantParameters.t > mainSys.configParams.launchRail.t_launchRailButtonCleared, 1, 'first');

fprintf("Avg. Thrust-to-Weight: %.1f\n", mean(resultantParameters.params.ThrustToWeight(~isnan(resultantParameters.params.ThrustToWeight))));
fprintf("Thrust-to-Weight at LRE: %.1f\n", resultantParameters.params.ThrustToWeight(LREind));
fprintf("SSM on pad: %.2f\n", resultantParameters.params.SSM(1));
fprintf("SSM at LRE: %.2f\n", resultantParameters.params.SSM(LREind));

ascentStates = resultantStates.interpolate(0:0.005:mainSys.configParams.apogeeListener.t_trigger);
fprintf("Max G (ascent): %.1f\n", max(ascentStates.accelMag) / 9.8);

drogueOpeningStates = resultantStates.interpolate(mainSys.configParams.apogeeListener.t_trigger-0.5:0.001:mainSys.configParams.mainDeploymentListener.t_trigger-0.5);
fprintf("Max G (drogue): %.1f\n", max(drogueOpeningStates.accelMag) / 9.8);
mainOpeningStates = resultantStates.interpolate(mainSys.configParams.mainDeploymentListener.t_trigger-0.5:0.001:min(resultantStates.t(end - 10), mainSys.configParams.jettisonListener.t_trigger-0.5));
fprintf("Max G (main): %.1f\n", max(mainOpeningStates.accelMag) / 9.8);

drogueNames = ["Fore", "Aft"];
drogueMasses = [mainSys.configParams.sectionalMasses(1) sum(mainSys.configParams.sectionalMasses(2:3))];

if isfield(mainSys.configParams, 'jettisonedTotalMass') && mainSys.configParams.jettisonedTotalMass > 0
  drogueMasses(1) = drogueMasses(1) + mainSys.configParams.jettisonedTotalMass * uc.kg_to_lbm;
end

drogueKEs = mainOpeningStates.yd(1)^2 * 0.5 * uc.J_to_ftlbf * uc.lbm_to_kg .* drogueMasses;

fprintf('Drogue:          [%.2f lb, %.2f lb]\n', drogueMasses(1), drogueMasses(2));

sectionNames = ["Fore", "Mid", "Aft"];
sectionMasses = mainSys.configParams.sectionalMasses';

sectionMasses(1) = sectionMasses(1) - 0.65;

sectionKEs = resultantStates.yd(end - 10)^2 * 0.5 * uc.J_to_ftlbf * uc.lbm_to_kg .* sectionMasses;

descentTime = resultantStates.t(end) - mainSys.configParams.apogeeListener.t_trigger;

fprintf("Drogue KE: %s - %.1f lb-ft (65 bonus/75 limit)\n", [drogueNames; drogueKEs]);
fprintf('Main:            [%.2f lb, %.2f lb, %.2f lb]\n', sectionMasses(1), sectionMasses(2), sectionMasses(3));
fprintf("Landing KE: %s - %.1f lb-ft (65 bonus/75 limit)\n", [sectionNames; sectionKEs]);
fprintf("Landing vel: %.1f fps\n", -resultantStates.yd(end - 10) * uc.mps_to_fps);
fprintf("Descent Time: %.1f s (80/90)\n", descentTime);
fprintf("Drogue vel: %.3f fps\n", mainOpeningStates.yd(1) * uc.mps_to_fps);

fprintf("Drift from pad: %.0f ft\n", resultantStates.x(end) .* uc.m_to_ft);

apogeeState = mainSys.configParams.apogeeListener.systemStateAtTrigger;

fprintf("Drift from apogee: %.0f ft\n", (apogeeState.x - resultantStates.x(end)) .* uc.m_to_ft)
fprintf("Drift (nominal): %.0f ft\n", descentTime * mainSys.configParams.driftCalcWindSpeed * uc.mph_to_fps)
% return;
%%
[payloadSys, payloadStates, ~] = VADL.sims.payload(mainSys, resultantStates);

payloadStates.t = payloadStates.t + mainSys.configParams.jettisonListener.t_trigger;

return;