%% launchDaySensitivityAnalysis.m
% Sensitivity analysis for VADL launch vehicle under varying conditions
clear; clc; close all;
uc = DRSS.util.unitConv;

%% Configuration Parameters
% ballast_lbm = [0:0.1:2.3, 2.3:0.1:3.3]; % Ballast mass range [lbm]
% wind_speed_mph = [0, 5, 11, 15, 20]; % Wind speeds [mph]
% temps_F = [80, 90, 100]; % Temperature range [F]
% Monte Carlo
ballast_lbm = [0:0.1:2.2, 2.3:0.1:3.3]; % Ballast mass range [lbm]
wind_speed_mph = 0:2:20; % Wind speeds [mph]
temps_F = 80:5:100; % Temperature range [F]
numCases = length(ballast_lbm)*length(wind_speed_mph)*length(temps_F);

%% Preallocate Results Matrices
apogeeResults = zeros(length(ballast_lbm), length(wind_speed_mph), length(temps_F));
ssmResults = zeros(length(ballast_lbm), 1);

%% Main Simulation Loop
fprintf('Running %d simulation cases:\n', numCases);
progressBar = waitbar(0, 'Starting simulations...');

for bIdx = 1:length(ballast_lbm)
  % Configure ballast offset (VDF baseline = 2.3 lbm)
  ballastOffset = ballast_lbm(bIdx) - 2.3;

  % Base system configuration
  sysConfig = struct(...
    'ballastOffset', ballastOffset, ...
    'disableJettison', false, ...
    'noMotor', false);

  % Get system parameters (SSM is wind/temp independent)
  mainSys = VADL.config.getMainSys(sysConfig);
  ssmResults(bIdx) = mainSys.configParams.ssm;

  for wIdx = 1:length(wind_speed_mph)
    for tIdx = 1:length(temps_F)
      % Update environmental parameters
      sysConfig.windSpeed = wind_speed_mph(wIdx);
      sysConfig.launchSiteTemp = temps_F(tIdx);

      % Solve trajectory
      sys = VADL.config.getMainSys(sysConfig);
      solver = DRSS.solver.MatlabODESolver(sys);
      [states, ~] = solver.configureODE('RelTol',1e-3).setTimeSpan([0 22]).overrideODEFunc(@ode113).solve();
      apogeeResults(bIdx,wIdx,tIdx) = max(states.y)*uc.m_to_ft;

      % Update progress
      progress = ((bIdx-1)*length(wind_speed_mph)*length(temps_F) + ...
        (wIdx-1)*length(temps_F) + tIdx)/numCases;
      waitbar(progress, progressBar, sprintf('Progress: %.1f%%', progress*100));
    end
  end
end
close(progressBar);

%% Generate Apogee Sensitivity Plots
% Define a subdued color scheme using bluescale and line styles
windColors = winter(length(wind_speed_mph)); % Blue-green progression
windColors = flipud(windColors); % Darker colors for higher winds
lineStyles = {'-', '--', '-.', ':', '-'}; % Reduced style variations
tempLabels = {'80°F', '90°F', '100°F'};

for tIdx = 1:length(temps_F)
  fig = figure('Position', [100 100 800 600], ...
    'DefaultAxesFontName', 'Arial', ...
    'DefaultTextFontName', 'Arial', ...
    'Color', 'white');

  ax = axes(fig);
  hold on; grid on; box on;

  % Plot wind speed series with controlled color/styling
  for wIdx = 1:length(wind_speed_mph)
    plot(ballast_lbm, squeeze(apogeeResults(:,wIdx,tIdx)),...
      'Color', windColors(wIdx,:),...
      'LineStyle', '-', ... % Uniform line style
      'LineWidth', 2,...
      'Marker', 'none',...
      'DisplayName', sprintf('%d mph', wind_speed_mph(wIdx)));
  end

  % Formatting for technical presentation
  xlabel('Ballast Mass (lbm)', 'FontWeight', 'bold');
  ylabel('Apogee Altitude (ft)', 'FontWeight', 'bold');
  title(sprintf('Apogee Sensitivity at %s Ambient Temperature', tempLabels{tIdx}),...
    'FontSize', 14, 'FontWeight', 'bold');

  % Reference lines with subtle styling
  yline(4200, 'Color', [0.4 0.4 0.4], 'LineStyle', ':', ...
    'LineWidth', 1.5, 'DisplayName', 'Target (4200 ft)');
  yline(4000, 'Color', [0.4 0.4 0.4], 'LineStyle', '--', ...
    'LineWidth', 1.5, 'DisplayName', 'Minimum (4000 ft)');
  xline(2.3, 'Color', [0.7 0.2 0.2], 'LineStyle', '-', ...
    'LineWidth', 1.5, 'DisplayName', 'VDF Baseline');

  % Legend configuration
  leg = legend('Location', 'southwest',...
    'Box', 'off',...
    'FontSize', 10);
  leg.Title.String = 'Wind Speed';
  leg.Title.FontWeight = 'bold';

  % Axis formatting
  ax.XLim = [0 3.3];
  ax.YLim = [3800 4800];
  ax.GridColor = [0.7 0.7 0.7];
  ax.LineWidth = 1.2;
  ax.FontSize = 12;
  ax.XColor = [0.3 0.3 0.3];
  ax.YColor = [0.3 0.3 0.3];

  % Add context annotation
  text(0.1, 0.95, sprintf('Temp: %s', tempLabels{tIdx}),...
    'Units', 'normalized',...
    'FontSize', 10,...
    'BackgroundColor', [1 1 1 0.8],...
    'EdgeColor', [0.8 0.8 0.8]);

  % Add colorbar for wind speed
  colormap(windColors);
  c = colorbar('Ticks', linspace(0,1,length(wind_speed_mph)),...
    'TickLabels', arrayfun(@(x)sprintf('%d mph',x), wind_speed_mph, 'UniformOutput', false));
  c.Label.String = 'Wind Speed Intensity';
  c.Label.FontSize = 10;
  c.Label.FontWeight = 'bold';


  exportgraphics(fig, sprintf("360_ballast_v_apogee_%s.png", tempLabels{tIdx}), "Resolution", 600);
end
%% SSM Stability Analysis
figSSM = figure('Position', [100 100 600 400]);
plot(ballast_lbm, ssmResults, 'b-', 'LineWidth', 2, 'MarkerSize', 6, 'HandleVisibility', 'off');
grid on; hold on;
yline(2.0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Minimum 2.0 SSM');
xline(2.3, 'Color', [0.3 0.3 0.3], 'LineWidth', 2, 'LineStyle', '-',...
  'DisplayName', 'VDF Baseline');

xlabel('Ballast Mass [lbm]');
ylabel('Static Stability Margin [cal]');
title('Static Stability vs Ballast Mass');
legend('Location', 'best');
set(gca, 'FontSize', 12);
exportgraphics(figSSM, "360_ballast_v_ssm.png", "Resolution", 600);

%% Create unified figure with multi-variable encoding
fig = figure('Position', [100 100 1000 750],...
  'DefaultAxesFontName', 'Helvetica',...
  'DefaultTextFontSize', 12);

% Create main analysis axis
axMain = axes('Position', [0.12 0.15 0.7 0.75]);
hold(axMain, 'on');
grid(axMain, 'on');

% Visual encoding parameters
tempPalette = [0.16 0.29 0.55;  % 80°F - Deep blue
  0.20 0.60 0.40;   % 90°F - Emerald
  0.80 0.25 0.25];  % 100°F - Vermilion
windStyles = {'-', '--', '-.', ':', '-'};
windLevels = wind_speed_mph;

% Plot all combinations with systematic encoding
lineHandles = gobjects(length(windLevels), length(tempPalette));
for tIdx = 1:length(temps_F)
  for wIdx = 1:length(windLevels)
    plotData = squeeze(apogeeResults(:,wIdx,tIdx));
    lineHandles(wIdx,tIdx) = plot(axMain, ballast_lbm, plotData,...
      'Color', tempPalette(tIdx,:),...
      'LineStyle', windStyles{wIdx},...
      'LineWidth', 2.5,...
      'Marker', 'none');
  end
end

% Reference system
refLines = struct(...
  'y4200', yline(axMain, 4200, 'k-.', 'LineWidth', 1.8),...
  'y4000', yline(axMain, 4000, 'r-.', 'LineWidth', 1.8),...
  'x23', xline(axMain, 2.3, 'k:', 'LineWidth', 2));

% Axis configuration
set(axMain, 'XLim', [0 3.3],...
  'YLim', [floor(min(apogeeResults,[],'all')/100)*100,...
  ceil(max(apogeeResults,[],'all')/100)*100],...
  'GridAlpha', 0.2,...
  'LineWidth', 1.2);
xlabel(axMain, 'Ballast Mass (lbm)', 'FontWeight', 'bold');
ylabel(axMain, 'Apogee Altitude (ft)', 'FontWeight', 'bold');
title(axMain, 'Integrated Flight Performance Sensitivity',...
  'FontSize', 14, 'FontWeight', 'bold');

% Temperature legend (color coded)
axTemp = axes('Position', [0.83 0.60 0.15 0.30], 'Visible', 'off');
for tIdx = 1:length(temps_F)
  annotation('rectangle', [0.85 0.78-tIdx*0.08 0.03 0.03],...
    'FaceColor', tempPalette(tIdx,:),...
    'EdgeColor', 'none');
  annotation('textbox', [0.88 0.77-tIdx*0.08 0.1 0.05],...
    'String', sprintf('%d°F', temps_F(tIdx)),...
    'EdgeColor', 'none',...
    'FontSize', 10);
end

% Wind speed legend (line style)
axWind = axes('Position', [0.83 0.20 0.15 0.30], 'Visible', 'off');
for wIdx = 1:length(windLevels)
  annotation('line', [0.85 0.89], [0.46-wIdx*0.04 0.46-wIdx*0.04],...
    'Color', [0.3 0.3 0.3],...
    'LineStyle', windStyles{wIdx},...
    'LineWidth', 2);
  annotation('textbox', [0.90 0.43-wIdx*0.04 0.1 0.05],...
    'String', sprintf('%d mph', windLevels(wIdx)),...
    'EdgeColor', 'none',...
    'FontSize', 10);
end

% Reference legend
annotation('textbox', [0.82 0.05 0.15 0.10],...
  'String', {...
  sprintf('-- 4200 ft: Target altitude'),...
  sprintf('-- 4000 ft: NASA minimum'),...
  sprintf( ': 2.3 lbm: VDF baseline')},...
  'EdgeColor', 'none',...
  'FontSize', 9);

%% Probabilistic Apogee Metrics (Append this instead of distribution plot)
mu_wind = 11; sigma_wind = 3; % mph
mu_temp = 90; sigma_temp = 5; % °F
A_thresholds = [4000, 4200, 4400, 4600];

% Create parameter grid using NDGRID format
[W_grid,T_grid] = ndgrid(linspace(mu_wind-3*sigma_wind, mu_wind+3*sigma_wind, 25),...
                   linspace(mu_temp-3*sigma_temp, mu_temp+3*sigma_temp, 25));
Ballast_grid = 2.3*ones(size(W_grid)); % Fixed ballast dimension

% Create interpolant with proper dimension order
apogeeInterp = griddedInterpolant({ballast_lbm, wind_speed_mph, temps_F}, apogeeResults, 'linear');
A_grid = apogeeInterp(Ballast_grid, W_grid, T_grid);

% Calculate probabilities
prob_W = normpdf(W_grid, mu_wind, sigma_wind);
prob_T = normpdf(T_grid, mu_temp, sigma_temp);
joint_prob = prob_W .* prob_T;
joint_prob = joint_prob / sum(joint_prob(:)); 

% Calculate statistics
valid = ~isnan(A_grid);
mu_A = sum(A_grid(valid) .* joint_prob(valid));
var_A = sum((A_grid(valid)-mu_A).^2 .* joint_prob(valid));
sigma_A = sqrt(var_A);

% Calculate exceedance probabilities
P_exceed = zeros(size(A_thresholds));
for i = 1:length(A_thresholds)
    mask = (A_grid >= A_thresholds(i)) & valid;
    P_exceed(i) = sum(joint_prob(mask));
end

% NASA-Level Distribution Plot
fig = figure('Position', [100 100 800 500]);
ax = axes(fig);

% Kernel density estimate
[f,xi] = ksdensity(A_grid(valid));
plot(xi, f, 'LineWidth', 2, 'Color', 'black');
hold on; grid on;

% Normal distribution overlay
x = linspace(mu_A-4*sigma_A, mu_A+4*sigma_A, 300);
y = normpdf(x, mu_A, sigma_A);
plot(x, y, '--', 'Color', [0.85 0.325 0.098], 'LineWidth', 1.5);

% Threshold annotations
thresholds = [4000, 4200, 4400];
colors = [
  1 0 0,
  0 0 1,
  0 0 1
  ];
for i = 1:3
    xline(thresholds(i), 'Color', colors(i,:), 'LineWidth', 1.3,...
        'Label', sprintf(' %d ft', thresholds(i)), 'LabelOrientation','horizontal');
end

% Formatting
xlabel('Apogee Altitude (ft)');
ylabel('Probability Density');
title('Apogee Altitude Probability Distribution');
legend(sprintf('Monte Carlo Distribution (N=%d)', numCases), 'Normal Approximation',...
    'Location', 'west');
ax.FontSize = 12;
ax.XLim = [3800 4600];
ax.YLim = [0 max(f)*2.5];
exportgraphics(fig, "360_monte_carlo.png", "Resolution", 600);

%% Richardson Extrapolation for Error Quantification
% Define grid refinement levels
fine_grid = [25 25];  % N_wind x N_temp
coarse_grid = [13 13]; 

% Compute integrals at two resolutions
[E_A_fine, ~, P_fine] = computeIntegralMetrics(fine_grid, apogeeInterp);
[E_A_coarse, ~, P_coarse] = computeIntegralMetrics(coarse_grid, apogeeInterp);

% Richardson extrapolation (2nd order method)
richardson_factor = (fine_grid(1)/coarse_grid(1))^2; % Assuming h^2 convergence
E_A_rich = (richardson_factor*E_A_fine - E_A_coarse)/(richardson_factor - 1);
error_E_A = abs(E_A_fine - E_A_rich)/3;

% Repeat for exceedance probabilities 
P_rich = zeros(size(P_fine));
error_P = zeros(size(P_fine));
for i = 1:length(P_fine)
    P_rich(i) = (richardson_factor*P_fine(i) - P_coarse(i))/(richardson_factor - 1);
    error_P(i) = abs(P_fine(i) - P_rich(i))/3;
end

fprintf('\nRichardson Extrapolation Results:\n');
fprintf('--------------------------------\n');
fprintf('Metric\t\tFine Grid\tCoarse Grid\tExtrapolated\tError\n');
fprintf('E[A]\t\t%.1f\t\t%.1f\t\t%.1f\t\t%.1f (%.2f%%)\n',...
        E_A_fine, E_A_coarse, E_A_rich, error_E_A, 100*error_E_A/E_A_rich);
for i = 1:length(P_fine)
    fprintf('P(A≥%d)\t\t%.3f\t\t%.3f\t\t%.3f\t\t%.6f\n',...
            A_thresholds(i), P_fine(i), P_coarse(i), P_rich(i), error_P(i));
end

% Helper Function
function [E_A, sigma_A, P_exceed] = computeIntegralMetrics(grid_dims, apogeeInterp)
  mu_wind = 11; sigma_wind = 3; % mph
  mu_temp = 90; sigma_temp = 5; % °F

A_thresholds = [4000, 4200, 4400, 4600];
  % Create grid with specified dimensions
  [W_grid,T_grid] = ndgrid(linspace(mu_wind-3*sigma_wind, mu_wind+3*sigma_wind, grid_dims(1)),...
                   linspace(mu_temp-3*sigma_temp, mu_temp+3*sigma_temp, grid_dims(2)));
  
  % Interpolate apogee values
  A_grid = apogeeInterp(2.3*ones(size(W_grid)), W_grid, T_grid);
  
  % Compute probabilities
  prob_W = normpdf(W_grid, mu_wind, sigma_wind);
  prob_T = normpdf(T_grid, mu_temp, sigma_temp);
  joint_prob = prob_W .* prob_T;
  joint_prob = joint_prob / sum(joint_prob(:));
  
  % Calculate metrics
  valid = ~isnan(A_grid);
  E_A = sum(A_grid(valid) .* joint_prob(valid));
  var_A = sum((A_grid(valid)-E_A).^2 .* joint_prob(valid));
  sigma_A = sqrt(var_A);
  
  P_exceed = zeros(size(A_thresholds));
  for i = 1:length(A_thresholds)
      mask = (A_grid >= A_thresholds(i)) & valid;
      P_exceed(i) = sum(joint_prob(mask));
  end
end

%% Anderson-Darling Normality Test
% Generate empirical apogee distribution
A_clean = A_grid(:);

% Perform AD test
[~, p_value, ad_stat, cv] = adtest(A_clean, 'Distribution', 'normal');

% Q-Q Plot
fig = figure('Position', [100 100 800 400]);
subplot(1,2,1);
qqplot(A_clean);
grid on; title(''); 
xlabel('Normal Quantiles'); 
ylabel('Apogee Quantiles');
set(gca, 'FontSize', 10);

subplot(1,2,2);
histogram(A_clean, 'Normalization', 'pdf', 'BinEdges', 4100:50:4600);
hold on; grid on;
x = linspace(4100,4600,300);
plot(x, normpdf(x, mu_A, sigma_A), 'r', 'LineWidth', 1.5);
xlabel('Apogee Altitude (ft)');
ylabel('Probability Density');
legend('Empirical', 'Normal Fit');
set(gca, 'FontSize', 10);

fprintf('\nAnderson-Darling Test Results:\n');
fprintf('A² Statistic: %.3f (Critical @ α=0.05: %.3f)\n', ad_stat, cv);
fprintf('p-value: %.3f\n', p_value);