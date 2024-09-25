
uc = DRSS.util.unitConv;

%% Load up VADL configurations
sys = DRSS.core.sim.System("Main Rocket");

VADL.config.gravity(sys);
VADL.config.geometry(sys);
VADL.config.massGroups(sys);

VADL.config.parachutes(sys);
VADL.config.descentDrag(sys);
VADL.config.launchCode(sys);
VADL.config.eventsSetup(sys);
VADL.config.buildFullRocketSystem(sys);

ssm = (sys.configParams.rocketDynamics.aerodynamicProfile.CP - sys.cgX) / sys.configParams.rocketDynamics.aerodynamicProfile.D

% sys.overrideCGX(48.292 * uc.in_to_m);

%% Solve
solver = DRSS.solver.ODE45Solver(sys) ...
  .setCaptureResultantParameters(true) ... % Whether to capture time variant params (i.e., m, mdot, I), which slows down the solver drastically
  .setPrintPerformanceSummary(true) ...
  .configureODE('RelTol', 1e-9) ... % 1e-3 for prototyping, 1e-8 for reports
  .overrideODEFunc(@ode113); % ode113 (2x more space efficient/less frames, run at 1e-9 rel tol), ode45 (default), ode89 (high precision)

[resultantStates, resultantParameters] = solver.solve();

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

%% Plot theta over time

plot(resultantStates.t, rad2deg(resultantStates.theta))
xline(sys.configParams.launchRail.t_launchRailCGCleared);
xline(sys.configParams.apogeeListener.t_trigger);
xlim([0, sys.configParams.apogeeListener.t_trigger + 2]);