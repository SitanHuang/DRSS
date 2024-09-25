function sys = launchCode(sys)

uc = DRSS.util.unitConv;

% Launch Site

sys ...
  .setLaunchSiteElevation(800 * uc.ft_to_m) ... set launch site elevation above mean sea level
  .setLaunchSiteWindSpeed(11 * uc.mph_to_mps) ... reference (base) wind velocity
  ;

% Launch Rail

sys.configParams.launchRail = DRSS.core.dynamics.LaunchRail() ...
  .setLaunchRailAngleDeg(5) ...
  .setLaunchRailLength(12 * 12 * uc.in_to_m) ...
  .setLaunchRailButtonLoc(61 * uc.in_to_m) ...
  .setLaunchRailExitVelocityMethod(DRSS.core.dynamics.LaunchExitVelocityMethods.RAIL_BUTTON_CROSSES_RAIL_TIP) ...
  .bindToGravityDynamics(sys.configParams.gravityDynamics);

% Motor

sys.configParams.motorDynamics = DRSS.core.dynamics.Motor( ...
  fullfile(fileparts(mfilename('fullpath')), '../Motor Data/L1400.csv'), ...
  "L1400", @VADL.vadl_motor_database);

motor = sys.configParams.motorDynamics.genMotorMass(); % the motor Mass is managed by this motorDynamics
sys.appendChild(motor);

% Deployments

sys.configParams.deployMainAltitude = 600 * uc.ft_to_m;
sys.configParams.jettisonAltitude = 300 * uc.ft_to_m;