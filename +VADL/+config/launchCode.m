function sys = launchCode(sys)

uc = DRSS.util.unitConv;

% Launch Site

if ~isfield(sys.configParams, 'windSpeed')
  sys.configParams.windSpeed = 11;
end
if ~isfield(sys.configParams, 'launchAngle')
  sys.configParams.launchAngle = 5;
end
if ~isfield(sys.configParams, 'launchSiteElevation')
  sys.configParams.launchSiteElevation = 600;
end
if ~isfield(sys.configParams, 'launchSiteTemp')
  sys.configParams.launchSiteTemp = 90;
end

sys ...
  .setLaunchSiteElevation(sys.configParams.launchSiteElevation * uc.ft_to_m) ... lebanon, TN - elevation above mean sea level
  .setLaunchSiteTemp((sys.configParams.launchSiteTemp - 32) * 5 / 9) ... launch site temp in deg C
  .setLaunchSiteWindSpeed(sys.configParams.windSpeed * uc.mph_to_mps) ... reference (base) wind velocity
  ;

% Launch Rail

sys.configParams.launchRail = DRSS.core.dynamics.LaunchRail() ...
  .setLaunchRailAngleDeg(sys.configParams.launchAngle) ...
  .setLaunchRailLength((12 - 1) * 12 * uc.in_to_m) ... one foot reserved for motor ground clearance
  .setLaunchRailButtonLoc(77 * uc.in_to_m) ...
  .setLaunchRailExitVelocityMethod(DRSS.core.dynamics.LaunchExitVelocityMethods.RAIL_BUTTON_CROSSES_RAIL_TIP) ...
  .bindToGravityDynamics(sys.configParams.gravityDynamics);

% Motor

motorOverride = "L1720";

if isfield(sys.configParams, "motorOverride")
  motorOverride = sys.configParams.motorOverride;
end

sys.configParams.motorDynamics = DRSS.core.dynamics.Motor( ...
  fullfile(fileparts(mfilename('fullpath')), sprintf('../Motor Data/%s.csv', motorOverride)), ...
  motorOverride, @VADL.vadl_motor_database);

if ~isfield(sys.configParams, 'motorLossFactor')
  sys.configParams.motorLossFactor = 0.965; % L1720
end

sys.configParams.motorDynamics.setLossFactor(sys.configParams.motorLossFactor);

motor = sys.configParams.motorDynamics.genMotorMass(); % the motor Mass is managed by this motorDynamics

if ~(isfield(sys.configParams, 'noMotor') && sys.configParams.noMotor)
  sys.appendChild(motor);
end

if isfield(sys.configParams, 'applyMassOffsetAtWetCG') && sys.configParams.applyMassOffsetAtWetCG
  sys.appendChild(DRSS.core.obj.Mass("Extra Mass Applied at Wet CG") ...
      .setM(sys.configParams.applyMassOffsetAtWetCG * uc.lbm_to_kg) ...
      .setLen(0) ...
      .setOffset(-sys.len + sys.cgX) ...
      .setCGX(0) ...
      .setInertialGeometry(struct( ...
        'Ri', sys.configParams.rocket_diameter / 2 - 0.085 * uc.in_to_m, ...
        'Ro', sys.configParams.rocket_diameter / 2 ...
      )));
end

% Deployments

% BELOW IS USED ONLY FOR CERTAIN VEHICLE CONFIGURATIONS (e.g., your IMU is not supported by drogue until main chute shock cord tightens)
% sys.configParams.deployMainAltitude = (750) * uc.ft_to_m;
% sys.configParams.mainOpeningAltitude = (750 - 125) * uc.ft_to_m; % 25ft shock cord takes 150 ft altitude difference at 100 fps drogue speed; 125 ft for 80 fps drogue

sys.configParams.deployMainAltitude = (750 - 25) * uc.ft_to_m;
sys.configParams.mainOpeningAltitude = (750 - 25 - 1) * uc.ft_to_m;

sys.configParams.jettisonAltitude = 300 * uc.ft_to_m;