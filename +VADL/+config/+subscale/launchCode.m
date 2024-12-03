function sys = launchCode(sys)

uc = DRSS.util.unitConv;

% Launch Site

if ~isfield(sys.configParams, 'windSpeed')
  sys.configParams.windSpeed = 11;
end
if ~isfield(sys.configParams, 'launchAngle')
  sys.configParams.launchAngle = 5;
end

sys ...
  .setLaunchSiteElevation(528 * uc.ft_to_m) ... set launch site elevation above mean sea level
  .setLaunchSiteWindSpeed(sys.configParams.windSpeed * uc.mph_to_mps) ... reference (base) wind velocity
  ;

% Launch Rail

sys.configParams.launchRail = DRSS.core.dynamics.LaunchRail() ...
  .setLaunchRailAngleDeg(sys.configParams.launchAngle) ...
  .setLaunchRailLength((8 - 1) * 12 * uc.in_to_m) ... one foot reserved for motor ground clearance
  .setLaunchRailButtonLoc(45 * uc.in_to_m) ...
  .setLaunchRailExitVelocityMethod(DRSS.core.dynamics.LaunchExitVelocityMethods.RAIL_BUTTON_CROSSES_RAIL_TIP) ...
  .bindToGravityDynamics(sys.configParams.gravityDynamics);

% Motor

motorOverride = "I366";

if isfield(sys.configParams, "motorOverride")
  motorOverride = sys.configParams.motorOverride;
end

sys.configParams.motorDynamics = DRSS.core.dynamics.Motor( ...
  fullfile(fileparts(mfilename('fullpath')), sprintf('../../Motor Data/%s.csv', motorOverride)), ...
  motorOverride, @VADL.vadl_motor_database);

if ~isfield(sys.configParams, 'motorLossFactor')
  sys.configParams.motorLossFactor = 0.965; % I366
end

sys.configParams.motorDynamics.setLossFactor(sys.configParams.motorLossFactor);

motor = sys.configParams.motorDynamics.genMotorMass(); % the motor Mass is managed by this motorDynamics
sys.appendChild(motor);