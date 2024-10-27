uc = DRSS.util.unitConv;

files = dir('./+VADL/Motor Data/L*.csv');

for k = 1:length(files)
  filename = files(k).name;
  motortype = filename(1:end-4);

  % disp(['## Processing motor type: ', motortype]);

  sys = VADL.config.getMainSys(struct( ...
  'motorOverride', motortype, ...
  'motorLossFactor', (1 - 0.02), ...
  'windSpeed', 0));
  %% Solve
  solver = DRSS.solver.MatlabODESolver(sys) ...
    .setCaptureResultantParameters(true) ...
    .configureODE('RelTol', 1e-3, 'AbsTol', 1e-4) ...
    .setPrintPerformanceSummary(false) ...
    .overrideODEFunc(@ode45);

  [resultantStates, resultantParameters] = solver.solve();

  ascentStates = resultantStates.interpolate(0:0.005:sys.configParams.apogeeListener.t_trigger);

  fprintf('%s, %.0f ft, descent time %.1f s, %.1f lb-ft, %.1f G, %.1f LRE, avg thrustToWeight: %.2f, drift from pad %.0f ft\n', ...
    motortype, ...
    max(resultantStates.y * uc.m_to_ft), ...
    resultantStates.t(end) - sys.configParams.apogeeListener.t_trigger, ...
    resultantStates.yd(end)^2 * 0.5 * 11 * uc.lbm_to_kg * uc.J_to_ftlbf, ...
    max(ascentStates.accelMag) / 9.8, ...
    sys.configParams.launchRail.v_launchRailExit .* uc.mps_to_fps, ...
    mean(resultantParameters.params.ThrustToWeight(~isnan(resultantParameters.params.ThrustToWeight))), ...
    resultantStates.x(end) .* uc.m_to_ft);
end