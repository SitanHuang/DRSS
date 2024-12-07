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
  sys.configParams.launchSiteElevation = 528;
end
if ~isfield(sys.configParams, 'launchSiteTemp')
  sys.configParams.launchSiteTemp = 45;
end

sys ...
  .setLaunchSiteElevation(sys.configParams.launchSiteElevation * uc.ft_to_m) ... lebanon, TN - elevation above mean sea level
  .setLaunchSiteTemp((sys.configParams.launchSiteTemp - 32) * 5 / 9) ... launch site temp in deg C
  .setLaunchSiteWindSpeed(sys.configParams.windSpeed * uc.mph_to_mps) ... reference (base) wind velocity
  ;

% Launch Rail

sys.configParams.launchRail = DRSS.core.dynamics.LaunchRail() ...
  .setLaunchRailAngleDeg(sys.configParams.launchAngle) ...
  ... .setLaunchRailLength((12 - 1) * 12 * uc.in_to_m) ... one foot reserved for motor ground clearance
  .setLaunchRailLength(12 * 12 * uc.in_to_m) ...
  .setLaunchRailButtonLoc((37 + 54) / 2 * uc.in_to_m) ...
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