% % Define wind speeds to simulate
% windSpeeds = [0, 5, 10, 15, 20, -5, -10, -15, -20];
% 
% % Initialize figure for plotting
% fig = figure;
% hold on;
% title("Flight Track");
% ylabel("Altitude [ft]");
% xlabel("Drift [ft]");
% grid on;
% 
% % Set line width for plotting
% lineWidth = 1;
% 
% % Define unit conversion from meters to feet
% m_to_ft = 3.28084;  % Since uc.m_to_ft is used in your code
% 
% % Define colors for headwinds (positive wind speeds) and tailwinds (negative wind speeds)
% colorsHead = {'#FFC0C0', '#FF8080', '#FF4040', '#FF0000', '#800000'}; % Light to dark red
% windSpeedsHead = [0, 5, 10, 15, 20];
% 
% colorsTail = {'#C0C0FF', '#8080FF', '#4040FF', '#0000FF', '#000080'}; % Light to dark blue
% windSpeedsTail = [0, -5, -10, -15, -20];
% 
% % Initialize legend entries
% legendEntries = {};
% 
% % Loop over each wind speed to simulate and plot
% for i = 1:length(windSpeeds)
%   windSpeed = windSpeeds(i);
% 
%   % Determine color based on wind speed
%   if windSpeed >= 0
%     % Headwind
%     idx = find(windSpeedsHead == windSpeed);
%     color = colorsHead{idx};
%     windType = 'Headwind';
%   else
%     % Tailwind
%     idx = find(windSpeedsTail == windSpeed);
%     color = colorsTail{idx};
%     windType = 'Tailwind';
%   end
% 
%   % Simulate rocket flight
%   mainSys = VADL.config.getMainSys(struct( ...
%     'motorLossFactor', (1 - 0.02), ...
%     'windSpeed', windSpeed));
% 
%   solver = DRSS.solver.MatlabODESolver(mainSys) ...
%     .setCaptureResultantParameters(true) ...
%     .configureODE('RelTol', 1e-3, 'AbsTol', 1e-4) ...
%     .overrideODEFunc(@ode45);
% 
%   [resultantStates, resultantParameters] = solver.solve();
% 
%   % Simulate payload (lander) flight
%   [payloadSys, payloadStates, ~] = VADL.sims.payload(mainSys, resultantStates);
%   payloadStates.t = payloadStates.t + mainSys.configParams.jettisonListener.t_trigger;
% 
%   % Convert positions from meters to feet
%   x_vehicle = resultantStates.x * m_to_ft;
%   y_vehicle = resultantStates.y * m_to_ft;
%   x_payload = payloadStates.x * m_to_ft;
%   y_payload = payloadStates.y * m_to_ft;
% 
%   % Plot vehicle trajectory
%   plot(x_vehicle, y_vehicle, 'Color', color, 'LineWidth', lineWidth);
% 
%   % Plot payload trajectory with dashed line
%   plot(x_payload, y_payload, 'Color', color, 'LineStyle', '--', 'LineWidth', lineWidth);
% 
%   % Add entries to legend
%   legendEntries{end+1} = sprintf('Vehicle, %s %d mph', windType, abs(windSpeed));
%   legendEntries{end+1} = sprintf('Payload, %s %d mph', windType, abs(windSpeed));
% end
% 
% % Add legend to the plot
% l = legend(legendEntries, 'Location', 'bestoutside');
% l.FontName = 'Consolas';
% 
% % Set plot limits
% xlim([-2000 3500]);
% ylim([0 4500]);


% Define wind speeds to simulate (in m/s)
windSpeeds_mph = [0, 5, 10, 15, 20, -5, -10, -15, -20];

% Initialize figure for plotting
fig = figure;
hold on;
title("Flight Track");
ylabel("Altitude [ft]");
xlabel("Drift [ft]");
grid on;

% Set line width for plotting
lineWidth = 1;

% Define unit conversion from meters to feet
m_to_ft = 3.28084;  % Since uc.m_to_ft is used in your code

% Define colors for headwinds (positive wind speeds) and tailwinds (negative wind speeds)
colorsHead = {'#FFC0C0', '#FF8080', '#FF4040', '#FF0000', '#800000'}; % Light to dark red
windSpeedsHead = [0, 5, 10, 15, 20];

colorsTail = {'#C0C0FF', '#8080FF', '#4040FF', '#0000FF', '#000080'}; % Light to dark blue
windSpeedsTail = [0, -5, -10, -15, -20];

% Initialize legend entries
legendEntries = {};

% Initialize table data
tableData = [];

% Loop over each wind speed to simulate and plot
for i = 1:length(windSpeeds_mph)
    windSpeed_mph = windSpeeds_mph(i);

    % Determine color based on wind speed
    if windSpeed_mph >= 0
        % Headwind
        idx = find(windSpeedsHead == windSpeed_mph);
        color = colorsHead{idx};
        windType = 'Headwind';
    else
        % Tailwind
        idx = find(windSpeedsTail == windSpeed_mph);
        color = colorsTail{idx};
        windType = 'Tailwind';
    end

    % Simulate rocket flight
    mainSys = VADL.config.getMainSys(struct( ...
        'motorLossFactor', (1 - 0.02), ...
        'windSpeed', windSpeed_mph));

    solver = DRSS.solver.MatlabODESolver(mainSys) ...
        .setCaptureResultantParameters(true) ...
        .configureODE('RelTol', 1e-3, 'AbsTol', 1e-4) ...
        .overrideODEFunc(@ode45);

    [resultantStates, resultantParameters] = solver.solve();

    % Get apogee state and time
    apogeeState = mainSys.configParams.apogeeListener.systemStateAtTrigger;
    t_apogee = mainSys.configParams.apogeeListener.t_trigger;

    % Simulate payload (lander) flight
    [payloadSys, payloadStates, ~] = VADL.sims.payload(mainSys, resultantStates);
    payloadStates.t = payloadStates.t + t_apogee;

    % Convert positions from meters to feet
    x_vehicle = resultantStates.x * m_to_ft;
    y_vehicle = resultantStates.y * m_to_ft;
    x_payload = payloadStates.x * m_to_ft;
    y_payload = payloadStates.y * m_to_ft;
    x_apogee = apogeeState.x * m_to_ft;

    % Plot vehicle trajectory
    plot(x_vehicle, y_vehicle, 'Color', color, 'LineWidth', lineWidth);

    % Plot payload trajectory with dashed line
    plot(x_payload, y_payload, 'Color', color, 'LineStyle', '--', 'LineWidth', lineWidth);

    % Add entries to legend
    legendEntries{end+1} = sprintf('Vehicle, %s %.1f mph', windType, abs(windSpeed_mph));
    legendEntries{end+1} = sprintf('Payload, %s %.1f mph', windType, abs(windSpeed_mph));

    % Calculate required statistics
    % Payload drift from apogee
    payloadDriftFromApogee = abs(payloadStates.x(end) - apogeeState.x) * m_to_ft;
    % Payload drift from pad
    payloadDriftFromPad = abs(payloadStates.x(end)) * m_to_ft;
    % Rocket drift from apogee
    rocketDriftFromApogee = abs(resultantStates.x(end) - apogeeState.x) * m_to_ft;
    % Rocket drift from pad
    rocketDriftFromPad = abs(resultantStates.x(end)) * m_to_ft;
    % Rocket descent time
    rocketDescentTime = resultantStates.t(end) - t_apogee;

    % Store the data in the table
    tableData = [tableData; {sprintf('%s: %.1f mph', windType, abs(windSpeed_mph)), ...
        payloadDriftFromApogee, payloadDriftFromPad, rocketDriftFromApogee, rocketDriftFromPad, rocketDescentTime}];
end

% Add legend to the plot
l = legend(legendEntries, 'Location', 'bestoutside');
l.FontName = 'Consolas';

% Set plot limits
xlim([-2000 3500]);
ylim([0 4500]);

% Create a table with the data
tableVarNames = {'WindSpeed', 'PayloadDriftFromApogee_ft', 'PayloadDriftFromPad_ft', 'RocketDriftFromApogee_ft', 'RocketDriftFromPad_ft', 'RocketDescentTime_s'};
T = cell2table(tableData, 'VariableNames', tableVarNames);

% Display the table
disp(T);
