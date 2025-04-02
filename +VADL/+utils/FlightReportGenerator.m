classdef FlightReportGenerator
  properties
    Config              % The input configuration struct
    MainSys             % Main system object from configuration
    Solver              % ODE solver object
    ResultantStates     % Simulation state results
    ResultantParameters % Simulation parameter results
    Metrics             % Calculated flight metrics
  end

  methods
    function obj = FlightReportGenerator(config)
      % Constructor accepts the configuration struct
      if nargin > 0
        obj.Config = config;
        obj.MainSys = VADL.config.getMainSys(config);
      end
    end

    function obj = runSimulation(obj)
      % Run the simulation if motor is enabled; otherwise, warn and exit.
      if isfield(obj.MainSys.configParams, 'noMotor') && obj.MainSys.configParams.noMotor
        warning('Motor disabled. Simulation aborted.');
        return;
      end

      % Set up the ODE solver with performance options
      obj.Solver = DRSS.solver.MatlabODESolver(obj.MainSys) ...
        .setCaptureResultantParameters(true) ...
        .setPrintPerformanceSummary(true) ...
        .configureODE('RelTol', 1e-4, 'MaxStep', 0.1) ...
        .overrideODEFunc(@ode15s);

      % Run the simulation
      [obj.ResultantStates, obj.ResultantParameters] = obj.Solver.solve();
    end

    function obj = computeMetrics(obj)
      uc = DRSS.util.unitConv;

      % Pre-compute some common indices and interpolated states
      LREind = find(obj.ResultantStates.t > obj.MainSys.configParams.launchRail.t_launchRailButtonCleared, 1, 'first');
      ascentStates = obj.ResultantStates.interpolate(0:0.005:obj.MainSys.configParams.apogeeListener.t_trigger);
      drogueOpeningStates = obj.ResultantStates.interpolate(...
        obj.MainSys.configParams.apogeeListener.t_trigger-0.5:0.001:obj.MainSys.configParams.mainDeploymentListener.t_trigger-0.5);
      mainOpeningStates = obj.ResultantStates.interpolate(...
        obj.MainSys.configParams.mainDeploymentListener.t_trigger-0.5:0.001:min(obj.ResultantStates.t(end-10), obj.MainSys.configParams.jettisonListener.t_trigger-0.5));

      % Calculate metrics and store in a structure
      metrics = struct();
      metrics.SSM_range = [obj.MainSys.configParams.rocketDynamics.ssm_min, ...
        obj.MainSys.configParams.rocketDynamics.ssm_max];
      metrics.CDr_range = [obj.MainSys.configParams.rocketDynamics.cdr_min, ...
        obj.MainSys.configParams.rocketDynamics.cdr_max];
      metrics.CAr_range = [max(obj.ResultantParameters.params.CAr), min(obj.ResultantParameters.params.CAr)];
      metrics.CP = [obj.MainSys.configParams.rocketDynamics.cp_calc_min * uc.m_to_in, ...
        obj.MainSys.configParams.rocketDynamics.cp_calc_max * uc.m_to_in, ...
        obj.MainSys.configParams.rocketDynamics.cp_calc_0 * uc.m_to_in];
      metrics.apogee = max(obj.ResultantStates.y) * uc.m_to_ft;
      metrics.LRE_velocity = obj.MainSys.configParams.launchRail.v_launchRailExit * uc.mps_to_fps;
      metrics.avg_TW = mean(obj.ResultantParameters.params.ThrustToWeight(~isnan(obj.ResultantParameters.params.ThrustToWeight)));
      metrics.TW_at_LRE = obj.ResultantParameters.params.ThrustToWeight(LREind);
      metrics.SSM_on_pad = obj.ResultantParameters.params.SSM(1);
      metrics.SSM_at_LRE = obj.ResultantParameters.params.SSM(LREind);
      metrics.maxG_ascent = max(ascentStates.accelMag) / 9.8;
      metrics.maxG_drogue = max(drogueOpeningStates.accelMag) / 9.8;
      metrics.maxG_main = max(mainOpeningStates.accelMag) / 9.8;

      % Calculate drogue and main stage masses and kinetic energies
      drogueMasses = [obj.MainSys.configParams.sectionalMasses(1), ...
        sum(obj.MainSys.configParams.sectionalMasses(2:3))];
      if isfield(obj.MainSys.configParams, 'jettisonedTotalMass') && obj.MainSys.configParams.jettisonedTotalMass > 0
        drogueMasses(1) = drogueMasses(1) + obj.MainSys.configParams.jettisonedTotalMass * uc.kg_to_lbm;
      end
      drogueKE = mainOpeningStates.yd(1)^2 * 0.5 * uc.J_to_ftlbf * uc.lbm_to_kg .* drogueMasses;
      metrics.drogueMasses = drogueMasses;
      metrics.drogueKE = drogueKE;

      sectionMasses = obj.MainSys.configParams.sectionalMasses';
      sectionKE = obj.ResultantStates.yd(end-10)^2 * 0.5 * uc.J_to_ftlbf * uc.lbm_to_kg .* sectionMasses;
      metrics.sectionMasses = sectionMasses;
      metrics.sectionKE = sectionKE;

      descentTime = obj.ResultantStates.t(end) - obj.MainSys.configParams.apogeeListener.t_trigger;
      metrics.descentTime = descentTime;
      metrics.landingVelocity = -obj.ResultantStates.yd(end-10) * uc.mps_to_fps;
      metrics.drogueVel = mainOpeningStates.yd(1) * uc.mps_to_fps;
      metrics.driftFromPad = obj.ResultantStates.x(end) * uc.m_to_ft;

      % Calculate drift metrics from apogee and nominal drift using wind speed
      apogeeState = obj.MainSys.configParams.apogeeListener.systemStateAtTrigger;
      metrics.driftFromApogee = (apogeeState.x - obj.ResultantStates.x(end)) * uc.m_to_ft;
      metrics.nominalDrift = (descentTime * obj.MainSys.launchSiteWindSpeed) * uc.m_to_ft;

      obj.Metrics = metrics;
    end

    function printReport(obj)
      uc = DRSS.util.unitConv;

      fprintf('Vehicle mass:  %.2f lb\n', obj.MainSys.m / uc.lbm_to_kg);
      if ~(isfield(obj.MainSys.configParams, 'noMotor') && obj.MainSys.configParams.noMotor)
        motor_param = obj.MainSys.configParams.motorDynamics.motor_params;
        fprintf('       (dry):  %.2f lb\n', (obj.MainSys.m - motor_param.m_prop0) / uc.lbm_to_kg);
        fprintf('  (no motor):  %.2f lb\n', (obj.MainSys.m - motor_param.m_prop0 - motor_param.m0) / uc.lbm_to_kg);
      end
      fprintf('Vehicle len:   %.2f in\n', obj.MainSys.len * uc.m_to_in);
      fprintf('Vehicle CGx:   %.2f in\n', obj.MainSys.cgX * uc.m_to_in);
      fprintf('Vehicle SSM:   %.2f cal\n', obj.MainSys.configParams.ssm);

      fprintf("SSM: %.2f - %.2f\n", obj.Metrics.SSM_range(1), obj.Metrics.SSM_range(2));
      fprintf("CDr: %.2f - %.2f\n", obj.Metrics.CDr_range(1), obj.Metrics.CDr_range(2));
      fprintf("CAr: %.2f - %.2f\n", obj.Metrics.CAr_range(1), obj.Metrics.CAr_range(2));
      fprintf("CP: %.2f - %.2f, %.2f\n", obj.Metrics.CP(1), obj.Metrics.CP(2), obj.Metrics.CP(3));
      fprintf("Apogee: %.0f ft\n", obj.Metrics.apogee);
      fprintf("LRE: %.1f fps\n", obj.Metrics.LRE_velocity);
      fprintf("Avg. Thrust-to-Weight: %.1f\n", obj.Metrics.avg_TW);
      fprintf("Thrust-to-Weight at LRE: %.1f\n", obj.Metrics.TW_at_LRE);
      fprintf("SSM on pad: %.2f\n", obj.Metrics.SSM_on_pad);
      fprintf("SSM at LRE: %.2f\n", obj.Metrics.SSM_at_LRE);
      fprintf("Max G (ascent): %.1f\n", obj.Metrics.maxG_ascent);
      fprintf("Max G (drogue): %.1f\n", obj.Metrics.maxG_drogue);
      fprintf("Max G (main): %.1f\n", obj.Metrics.maxG_main);

      fprintf('Drogue:          [%.2f lb, %.2f lb]\n', obj.Metrics.drogueMasses(1), obj.Metrics.drogueMasses(2));
      fprintf("Drogue KE: [%s] - %.1f lb-ft\n", num2str(obj.Metrics.drogueKE), obj.Metrics.drogueKE(end));
      fprintf('Main:            [%.2f lb, %.2f lb, %.2f lb]\n', ...
        obj.Metrics.sectionMasses(1), obj.Metrics.sectionMasses(2), obj.Metrics.sectionMasses(3));
      fprintf("Landing KE: [%s] - %.1f lb-ft\n", num2str(obj.Metrics.sectionKE), obj.Metrics.sectionKE(end));
      fprintf("Landing vel: %.1f fps\n", obj.Metrics.landingVelocity);
      fprintf("Descent Time: %.1f s\n", obj.Metrics.descentTime);
      fprintf("Drogue vel: %.3f fps\n", obj.Metrics.drogueVel);
      fprintf("Drift from pad: %.0f ft\n", obj.Metrics.driftFromPad);
      fprintf("Drift from apogee: %.0f ft\n", obj.Metrics.driftFromApogee);
      fprintf("Drift (nominal): %.0f ft\n", obj.Metrics.nominalDrift);
    end

    function report = generate(obj)
      % Convenience method to run the full workflow: simulation, metric computation, and reporting.
      obj = obj.runSimulation();
      obj = obj.computeMetrics();
      obj.printReport();

      % Return a structured report
      report = struct('States', obj.ResultantStates, ...
        'Parameters', obj.ResultantParameters, ...
        'Metrics', obj.Metrics);
    end
  end
end
