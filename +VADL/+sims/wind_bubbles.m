clear; clc; close all;

%% Set up wind speeds (all headwind, i.e., >= 0 mph)
windSpeeds_mph = [0.1, 5, 10, 15, 20];

%% Constants for unit conversion
mToFt    = 3.28084;   % meters to feet
mphToMps = 0.44704;   % miles/hour to meters/second

%% Figure setup
fig = figure('Name','Recovery Zones (Headwinds 0-20 mph)','Color','w');
hold on; grid on; axis equal;

title('Maximum Theoretical Recovery Zones for 0–20 mph Headwinds');
xlabel('Distance (ft)');
ylabel('Distance (ft)');

% NASA limit circle
NASA_limit_ft = 2500;
theta = linspace(0, 2*pi, 360);
x_NASA = NASA_limit_ft * cos(theta);
y_NASA = NASA_limit_ft * sin(theta);
plot(x_NASA, y_NASA, 'r--', 'LineWidth', 4);
text(NASA_limit_ft, 0, sprintf('  %d ft NASA 3.11 Recovery Limit', NASA_limit_ft), ...
    'Color', 'r', 'VerticalAlignment','middle', 'FontWeight','bold');

%% For legend & table
legendEntries = {"2500 ft NASA 3.11 Recovery Limit"};
tableData     = {};
tableVarNames = { ...
    'WindSpeed_mph',...
    'VehicleDescentTime_s',...
    'VehicleDrift_ft',...
    'PayloadDescentTime_s',...
    'PayloadDrift_ft'};

% Use a colormap or your own color set; for simplicity, we use `lines()`
cmap = lines(length(windSpeeds_mph));

for i = 1:length(windSpeeds_mph)
    %% Configure system for each wind speed
    wind_mph = windSpeeds_mph(i);
    mainSys = VADL.config.getMainSys(struct( ...
        'disableJettison', false, ...
        'windSpeed', wind_mph));  % still in mph per your config

    solver = DRSS.solver.MatlabODESolver(mainSys) ...
        .setCaptureResultantParameters(true);

    [resultantStates, resultantParameters] = solver.solve();

    % Vehicle descent time (end of flight minus time of apogee)
    t_apogee       = mainSys.configParams.apogeeListener.t_trigger;
    totalTime      = resultantStates.t(end);
    vehicleDescentTime_s = totalTime - t_apogee;

    % "Simpler" vehicle drift in ft
    % wind (m/s) = mphToMps * wind_mph
    % drift [ft] = descentTime [s] * wind [m/s] * mToFt
    wind_mps = wind_mph * mphToMps;
    vehicleDrift_ft = vehicleDescentTime_s * wind_mps * mToFt;

    % Solve for payload flight
    [payloadSys, payloadStates, ~] = VADL.sims.payload(mainSys, resultantStates);
    jettisonT  = mainSys.configParams.jettisonListener.t_trigger;
    payloadDescentTime_s = (payloadStates.t(end) - payloadStates.t(1)) ...
                           + (jettisonT - t_apogee);

    payloadDrift_ft = payloadDescentTime_s * wind_mps * mToFt;

    %% Plot circles for vehicle/payload drift
    colorThis = cmap(i,:);
    rVehicle  = vehicleDrift_ft;
    rPayload  = payloadDrift_ft;
    
    x_circV = rVehicle * cos(theta);
    y_circV = rVehicle * sin(theta);
    plot(x_circV, y_circV, 'Color', colorThis, 'LineWidth', 1.5);

    x_circP = rPayload * cos(theta);
    y_circP = rPayload * sin(theta);
    plot(x_circP, y_circP, '--', 'Color', colorThis, 'LineWidth', 1.5);

    %% Place text labels at an angle to avoid clutter
    % We'll just pick an angle offset that depends on i.
    % For example, for the vehicle we can do angleVehicle = 30 + 45*(i-1),
    % and for the payload add +15 degrees from there.
    angleVehicle_deg = 30 + 45*(i-1);
    anglePayload_deg = angleVehicle_deg + 15;

    % Convert degrees to radians for cos() / sin()
    angV = deg2rad(angleVehicle_deg);
    angP = deg2rad(anglePayload_deg);

    % Coordinates at those angles
    x_labelV = rVehicle * cos(angV);
    y_labelV = rVehicle * sin(angV);
    x_labelP = rPayload * cos(angP);
    y_labelP = rPayload * sin(angP);

    % if wind_mph > 0
    %   % Plot text
    %   text(x_labelV, y_labelV, ...
    %        sprintf(' %.0f ft', rVehicle, wind_mph), ...
    %        'Color', colorThis, 'VerticalAlignment','middle', ...
    %        'FontWeight', 'bold', 'BackgroundColor', 'Black');
    % 
    %   text(x_labelP, y_labelP, ...
    %        sprintf(' %.0f ft ', rPayload, wind_mph), ...
    %        'Color', colorThis, 'VerticalAlignment','middle', ...
    %        'FontAngle', 'italic', 'BackgroundColor', 'Black');
    % end

    % Add legend entries
    legendEntries{end+1} = sprintf('Drift=%.0f ft; Vehicle @ %g mph', rVehicle, wind_mph);
    legendEntries{end+1} = sprintf('Drift=%.0f ft; Payload @ %g mph', rPayload, wind_mph);

    %% Collect data into table
    tableData(end+1, :) = { ...
        wind_mph, ...
        vehicleDescentTime_s, ...
        vehicleDrift_ft, ...
        payloadDescentTime_s, ...
        payloadDrift_ft};
end

% Final Legend
legend(legendEntries, 'Location', 'bestoutside');

% Some axis limits (optional, or adjust as you see fit)
axis([-3000 3000 -3000 3000]);

%% Convert tableData to table and display
T = cell2table(tableData, 'VariableNames', tableVarNames);
disp(T);

%% Save figure as needed
print(fig, '360_wind_bubbles.png', '-dpng', '-r600');
