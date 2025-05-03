clear; clc; close all;

uc = DRSS.util.unitConv;

%%
fig = figure;
hold on;

for deployAltitude=[4000, 3500, 3000, nan]
  [resultantStates, resultantParameters, sys] = VADL.sims.solveMain(struct( ...
    'dragPlateEnable', true, ...
    'dragPlateTriggerFunc', @(ss) ss.y > (deployAltitude * uc.ft_to_m) ...
  ));

  if ~isnan(deployAltitude)
    name = sprintf("Drag plate at %.0f ft with 1s deploy time", deployAltitude);
  else
    name = "Drag plate off";
  end

  hPlot = plot(resultantStates.x .* uc.m_to_ft, resultantStates.y .* uc.m_to_ft, 'LineWidth', 1, 'DisplayName', name);

  if ~isnan(deployAltitude)
    deployState = sys.configParams.dragPlate.systemStateAtTrigger;
    scatter(deployState.x .* uc.m_to_ft, deployState.y .* uc.m_to_ft, 50, '*', 'MarkerFaceColor', hPlot.Color, 'MarkerEdgeColor', hPlot.Color, 'HandleVisibility', 'off');
  end
end

axis equal;
title("Flight Track - 0.6 Constant CD 0.5 ft^2 Drag Plate, Linear Deployment");
ylabel("Altitude [ft]")
xlabel("Drift [ft]");
grid on;
legend('Location', 'west')