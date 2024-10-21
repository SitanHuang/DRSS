uc = DRSS.util.unitConv;

%% Set up
sys = VADL.config.getMainSys;

fprintf('Vehicle mass:  %.2f lb\n', sys.m / uc.lbm_to_kg);
fprintf('Vehicle len:   %.2f in\n', sys.len * uc.m_to_in);
fprintf('Vehicle CGx:   %.2f in\n', sys.cgX * uc.m_to_in);
fprintf('Vehicle SSM:   %.2f cal\n', sys.configParams.ssm);

%% Solve

solver = DRSS.solver.MatlabODESolver(sys) ...
  .setCaptureResultantParameters(true);

[resultantStates, resultantParameters] = solver.solve();

%% Flight Report
fprintf("SSM: %.2f - %.2f\n", sys.configParams.rocketDynamics.ssm_min, sys.configParams.rocketDynamics.ssm_max);
fprintf("CDr: %.2f - %.2f\n", sys.configParams.rocketDynamics.cdr_min, sys.configParams.rocketDynamics.cdr_max);
fprintf("CAr: %.2f - %.2f\n", max(resultantParameters.params.CAr), min(resultantParameters.params.CAr));
fprintf("CP: %.2f - %.2f\n", sys.configParams.rocketDynamics.cp_calc_min, sys.configParams.rocketDynamics.cp_calc_max);
fprintf("Apogee: %.3f ft\n", max(resultantStates.y) .* uc.m_to_ft);
fprintf("LRE: %.3f fps\n", sys.configParams.launchRail.v_launchRailExit .* uc.mps_to_fps);
fprintf("Max G: %.2f\n", max((resultantStates.xdd.^2 + resultantStates.ydd.^2).^(1/2)) / 9.81);
fprintf("Drift: %.3f ft\n", resultantStates.x(end) .* uc.m_to_ft);
fprintf("Landing KE: %.3f lb-ft\n", resultantStates.yd(end)^2 * 0.5 * sys.configParams.recoveryBay.m * uc.J_to_ftlbf);
fprintf("Landing vel: %.3f fps\n", -resultantStates.yd(end) * uc.mps_to_fps);
fprintf("Descent Time: %.3f s\n", resultantStates.t(end) - sys.configParams.apogeeListener.t_trigger);

%% Plot
% plot(resultantParameters.t, resultantParameters.params.CAr);
% yyaxis right;
% plot(resultantParameters.t, resultantParameters.params.CDr);

plot(resultantStates.t, (resultantStates.xdd.^2 + resultantStates.ydd.^2).^(1/2) / 9.81);