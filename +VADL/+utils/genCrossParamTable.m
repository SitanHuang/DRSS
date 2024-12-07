function flightReports = genCrossParamTable(solveFunc, configParams, ...
  param1_values, param2_values, param1_name, param2_name, metrics_to_report, ...
  outputFilename)
    % Generates an HTML report of simulation results over a grid of parameters.
    % Inputs:
    %   solveFunc - function handle with signature [resultantStates, resultantParameters, sys] = solveFunc(configParams)
    %   configParams - Struct containing configuration parameters for the simulation
    %   param1_values - Array of values for the first parameter
    %   param2_values - Array of values for the second parameter
    %   param1_name - Name of the first parameter (e.g., 'windSpeed')
    %   param2_name - Name of the second parameter (e.g., 'launchAngle')
    %   metrics_to_report - Cell array of metric names to include in the report
    %   outputFilename - String, filename for the output HTML file
    % Output:
    %   flightReports - cell array of flight reports

    % Open file for writing
    fid = fopen(outputFilename, 'w');
    if fid == -1
      error('Could not open file %s for writing.', outputFilename);
    end

    % Write HTML header and style
    fprintf(fid, '<html>\n<head>\n<title>Simulation Results</title>\n');
    fprintf(fid, '<style>\n');
    fprintf(fid, 'table {border-collapse: collapse; width: 100%%;}\n');
    fprintf(fid, 'th, td {border: 1px solid black; padding: 5px; text-align: center;}\n');
    fprintf(fid, 'th {background-color: #f2f2f2;}\n');
    fprintf(fid, 'td {text-align: right;}\n');
    fprintf(fid, '</style>\n');
    fprintf(fid, '</head>\n<body>\n');
    fprintf(fid, '<table>\n');

    % Header row with param2_name and its values
    fprintf(fid, '<tr>\n');
    fprintf(fid, '<th rowspan="2">%s</th>\n', param1_name);
    fprintf(fid, '<th colspan="%d">%s</th>\n', length(param2_values), param2_name);
    fprintf(fid, '</tr>\n');

    % Second header row with param2 values
    fprintf(fid, '<tr>\n');
    for j = 1:length(param2_values)
      fprintf(fid, '<th>%g</th>\n', param2_values(j));
    end
    fprintf(fid, '</tr>\n');

    flightReports = cell(length(param1_values), length(param2_values));

    % Loop over param1 values
    for i = 1:length(param1_values)
      param1 = param1_values(i);

      % Start a new row for each param1 value
      fprintf(fid, '<tr>\n');
      fprintf(fid, '<th>%g</th>\n', param1);

      % Loop over param2 values
      for j = 1:length(param2_values)
        param2 = param2_values(j);

        % Set current parameter values
        configParams.(param1_name) = param1;
        configParams.(param2_name) = param2;

        % Run simulation with the given parameters
        flightReport = VADL.utils.genFlightReport(solveFunc, configParams);

        flightReports{i,j} = struct( ...
          'flightReport', flightReport, ...
          'configParams', configParams ...
        );

        % Build string with requested metrics
        metricStrings = cell(length(metrics_to_report), 1);
        for k = 1:length(metrics_to_report)
          metricName = metrics_to_report{k};
          if isfield(flightReport, metricName)
            metricValue = flightReport.(metricName);
            % 3 significant figures formatting
            if metricValue == 0
              formattedValue = '0'; % Handle zero explicitly
            else
              numDigits = max(0, 3 - floor(log10(abs(metricValue))) - 1); % Calculate decimal places needed
              formattedValue = sprintf(['%.', num2str(numDigits), 'f'], metricValue);
            end
            metricStrings{k} = sprintf('%s: %s', metricName, formattedValue);
          else
            metricStrings{k} = sprintf('%s: N/A', metricName);
          end
        end

        % Combine metric strings into one string separated by <br>
        cellString = strjoin(metricStrings, '<br>');
        % Write table cell
        fprintf(fid, '<td>%s</td>\n', cellString);
      end
      fprintf(fid, '</tr>\n');
    end

    % Close table and HTML tags
    fprintf(fid, '</table>\n</body>\n</html>');
    fclose(fid);

    fprintf('Simulation results have been written to %s\n', outputFilename);
end
