% Launch Site: Lebanon, TN 12/4/2024

%% Launch Code Reference Sheet
% Define parameter values for wind speed and launch angle
windSpeedMPH_values = [-11, -5, 0, 5, 11, 15, 20];
launchAngleDeg_values = [0, 2, 5, 7, 10];

% Define metrics to report
metrics_to_report = {'apogee_ft', 'maxTheoreticalDrift_ft'};

foreMass = 0.291+0.547+0.67+0.75+1.95+0.302;
aftDryMass = 11.6 - foreMass + 0.533; % 0.533 = motor dry mass

% Generate report in HTML
flightReports = VADL.utils.genCrossParamTable( ...
  @VADL.sims.solveSubscale, ...
  struct( ...
    'launchSiteTemp', 45, ... deg F
    'launchSiteElevation', 528, ... ft
    'landingKESectionMasses', [foreMass aftDryMass] ... lbm
  ), ...
  windSpeedMPH_values, launchAngleDeg_values, ...
  'windSpeed', 'launchAngle', ...
  metrics_to_report, ...
  '~out.html');

%% Baseline

baseline = flightReports{4, 3};
flightReport = baseline.flightReport;
fprintf("Baseline (%i mph, %i deg): \n", baseline.configParams.windSpeed, baseline.configParams.launchAngle);
fprintf("  LRE: %.1f fps\n", flightReport.launchRailExitVel_fps);
fprintf("  Avg Thrust-to-weight: %.1f\n", flightReport.avgThrustToWeight);
fprintf("  Apogee: %.0f ft\n", flightReport.apogee_ft);
fprintf("  Landing KEs: %.1f lb-ft\n", flightReport.landingKEs_lbft);
fprintf("  Landing vel: %.1f fps\n", flightReport.landingVel_fps);
fprintf("  Landing time (+apogee): %.1f s\n", flightReport.descentTime_s);