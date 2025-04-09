uc = DRSS.util.unitConv;

baseConfig = struct(...
  'disableJettison', false, ...
  'noMotor', false, ...
  'motorLossFactor', 0.965, ...
  'noBallast', true, ...
  'launchAngle', 5, ...
  'launchSiteElevation', 620, ...
  'launchSiteTemp', 82, ...
  'motorOverride', 'K1100', ...
  'launchRailEffectiveTravel', 13, ... % ft
  'windModelTurbulence', 1, ...
  'windModelFreq', 0.6 ...
);

%% Define the three scenarios
% wind speeds are in mph.
scenarios = { ...
  struct('date', 'April 5th', 'windSpeed', 14, 'windModelLowSpeed', 14*0.7, 'windModelHighSpeed', 30), ...
  struct('date', 'April 12th', 'windSpeed', 11, 'windModelLowSpeed', 11*0.7, 'windModelHighSpeed', 24), ...
  struct('date', 'April 13th', 'windSpeed', 14, 'windModelLowSpeed', 14*0.7, 'windModelHighSpeed', 32) ...
  };

%% Preallocate storage for results, states, and table rows
numScenarios = numel(scenarios);
results = cell(numScenarios,1);
vehicleStates = cell(numScenarios,1);
tableData = cell(numScenarios,5);

for i = 1:numScenarios
  % Merge the base config with scenario-specific wind parameters
  s = scenarios{i};
  scenarioConfig = baseConfig;
  scenarioConfig.windSpeed = s.windSpeed;
  scenarioConfig.windModelLowSpeed = s.windModelLowSpeed;
  scenarioConfig.windModelHighSpeed = s.windModelHighSpeed;

  reportGen = VADL.utils.FlightReportGenerator(scenarioConfig);
  report = reportGen.generate();
  results{i} = report;
  results{i}.MainSys = reportGen.MainSys;
  [results{i}.WindAvg, ~, ~, results{i}.WindStd] = report.Parameters.timeAveragedParam('windSpeed');

  vehicleStates{i} = report.States;

  apogee = report.Metrics.apogee;
  vehicleDrift = report.Metrics.nominalDrift;
  descentTime = report.Metrics.descentTime;

  apogeeToMain = reportGen.MainSys.configParams.main.t_enabled - reportGen.MainSys.configParams.apogeeListener.t_trigger;

  tableData(i,:) = { s.date, round(apogee, -1), sprintf("< %.0f", ceil(vehicleDrift/10)*10), round(descentTime, 1), round(apogeeToMain, 1) };
end

%% Build and display the summary table
T = cell2table(tableData, 'VariableNames', ...
  ["Scenario", "Apogee (ft)", "Vehicle Drift (ft)", "Vehicle Descent Time (s)", "Apogee to Main (s)"]);
disp(T);

%% Plot 1: Flight Tracks for all scenarios (vehicle and payload)
fig1 = figure;
colors = lines(numScenarios);  % Get distinct colors for each scenario
for i = 1:numScenarios
  plot(vehicleStates{i}.x .* uc.m_to_ft, vehicleStates{i}.y .* uc.m_to_ft, ...
    'Color', colors(i,:), 'LineWidth', 2, ...
    'DisplayName', sprintf( ...
      "%s - %.0f\\pm%.0f mph Wind", ...
      scenarios{i}.date, ...
      results{i}.WindAvg ./ uc.mph_to_mps, ...
      results{i}.WindStd ./ uc.mph_to_mps ...
  ));
  hold on;
end
title("Flight Track");
ylabel("Altitude [ft]");
xlabel("Drift [ft]");
grid on;
axis equal;
legend('Location','Best');

%% Plot 2: Rocket Pitch Angle for all scenarios
fig2 = figure;
for i = 1:numScenarios
  % yyaxis left;
  plot(vehicleStates{i}.t, rad2deg(vehicleStates{i}.theta), '-', ...
    'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf( ...
      "%s - %.0f\\pm%.0f mph Wind", ...
      scenarios{i}.date, ...
      results{i}.WindAvg ./ uc.mph_to_mps, ...
      results{i}.WindStd ./ uc.mph_to_mps ...
  ));
  hold on;
  % yyaxis right;
  % plot(results{i}.Parameters.t, results{i}.Parameters.windSpeed ./ uc.mph_to_mps, '--', 'LineWidth', 1, 'DisplayName', sprintf('%s - Simulated Wind', scenarios{i}.date))
end
title("Pitch Angle");
% ylabel("Wind Speed [mph]")
% yyaxis left;
ylabel("Pitch Angle [deg]");
xlabel("Time [s]");
% Use the apogee trigger from the first scenario to set x-axis limits
t_trigger = results{1}.MainSys.configParams.apogeeListener.t_trigger;
xlim([vehicleStates{1}.t(1) t_trigger]);
grid on;
legend('Location','Best');

%% Plot 3: Turbulence Wind Model
fig3 = figure;
plot(results{1}.Parameters.t - 10, results{1}.Parameters.windSpeed, 'b-', 'LineWidth', 1, 'DisplayName', sprintf('%s - Simulated Wind', scenarios{i}.date))
title("Sample Simulated Turbulence")
ylabel("Wind Speed [mph]")
xlabel("Time [s]")
xlim([0 25])
% xlim([vehicleStates{1}.t(1) t_trigger]);
grid on;