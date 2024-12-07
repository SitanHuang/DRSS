function flightReport = genFlightReport(solveFunc, configParams)
  % Runs the rocket simulation and generates a flight report struct.
  % Inputs:
  %   solveFunc - function handle with signature [resultantStates, resultantParameters, sys] = solveFunc(configParams)
  %   configParams - Struct passed to solve solveFunc
  % Output:
  %   flightReport - Struct containing flight metrics

  if nargin < 2
    configParams = struct();
  end

  uc = DRSS.util.unitConv;

  % Run simulation
  [resultantStates, resultantParameters, sys] = solveFunc(configParams);

  % Compute metrics and store in flightReport struct
  flightReport.vehicleMass_lb = sys.m / uc.lbm_to_kg;
  flightReport.vehicleLength_in = sys.len * uc.m_to_in;
  flightReport.vehicleCGx_in = sys.cgX * uc.m_to_in;
  flightReport.vehicleSSM_cal = sys.configParams.ssm;

  flightReport.SSM_min = sys.configParams.rocketDynamics.ssm_min;
  flightReport.SSM_max = sys.configParams.rocketDynamics.ssm_max;
  flightReport.CDr_min = sys.configParams.rocketDynamics.cdr_min;
  flightReport.CDr_max = sys.configParams.rocketDynamics.cdr_max;
  flightReport.CAr_min = min(resultantParameters.params.CAr);
  flightReport.CAr_max = max(resultantParameters.params.CAr);
  flightReport.CP_min = sys.configParams.rocketDynamics.cp_calc_min;
  flightReport.CP_max = sys.configParams.rocketDynamics.cp_calc_max;

  flightReport.apogee_ft = max(resultantStates.y) * uc.m_to_ft;
  flightReport.launchRailExitVel_fps = sys.configParams.launchRail.v_launchRailExit * uc.mps_to_fps;

  % Exclude NaN values in ThrustToWeight
  ThrustToWeight = resultantParameters.params.ThrustToWeight(~isnan(resultantParameters.params.ThrustToWeight));
  flightReport.avgThrustToWeight = mean(ThrustToWeight);

  flightReport.landingVel_fps = -resultantStates.yd(end - 10) * uc.mps_to_fps;

  descentTime = resultantStates.t(end) - sys.configParams.apogeeListener.t_trigger;
  flightReport.descentTime_s = descentTime;

  flightReport.drift_ft = resultantStates.x(end) * uc.m_to_ft;

  apogeeState = sys.configParams.apogeeListener.systemStateAtTrigger;

  flightReport.driftFromApogee_ft = (apogeeState.x - resultantStates.x(end)) * uc.m_to_ft;

  flightReport.nominalDrift_ft = descentTime * sys.launchSiteWindSpeed * uc.m_to_ft;

  % Compute maximum theoretical drift (max magnitude but preserve sign)
  drifts_ft = [flightReport.drift_ft, flightReport.driftFromApogee_ft, flightReport.nominalDrift_ft];

  [~, idx] = max(abs(drifts_ft));
  flightReport.maxTheoreticalDrift_ft = drifts_ft(idx);

  flightReport.landingKEs_lbft = [];

  if isfield(sys.configParams, 'landingKESectionMasses')
    flightReport.landingKEs_lbft = [];

    for massLb=sys.configParams.landingKESectionMasses
      massKg = massLb * uc.lbm_to_kg;
      velMps = -resultantStates.yd(end - 10);

      flightReport.landingKEs_lbft(end + 1) = 0.5 * massKg * velMps^2 * uc.J_to_ftlbf;
    end
  end
end
