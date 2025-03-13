function sys = eventsSetup(sys)

% Jettison Event: Set nose cone mass to 0
sys.configParams.jettisonEvent = DRSS.core.dynamics.Jettison() ...
  .trigger(sys.configParams.toBeJettisonedMasses);

% Apogee Listener: Detects when the rocket reaches apogee
sys.configParams.apogeeListener = DRSS.core.dynamics.events.Apogee() ...
  .setEnabledOnInit(false); % Prevent false triggers due to launch rail jitter

% Disable Ascent Dynamics Event: Disables ascent aerodynamics and apogee listener upon trigger
sys.configParams.disableAscentDynamics = DRSS.core.dynamics.events.TriggerOnEnable() ...
  .setDisableBoundDynamicsOnTrigger(true) ...
  .trigger(sys.configParams.rocketDynamics) ... % Disable ascent aerodynamics
  .trigger(sys.configParams.apogeeListener);    % Disable apogee listener

% Constant Theta Dynamics: Artificially remove angular velocity during descent
sys.configParams.constantThetadDynamics = DRSS.core.dynamics.ConstantThetadDynamics() ...
  .setEnabledOnInit(false) ...
  .setDampingConstant(10) ...
  .setTargetThetad(0);

% Main Deployment Listener: Triggers main parachute deployment at a specified altitude
sys.configParams.mainDeploymentListener = DRSS.core.dynamics.events.Altitude() ...
  .setEnabledOnInit(false) ...
  .setAlitude(sys.configParams.deployMainAltitude);

% Main Opening Listener: Triggers main parachute opening at a specified altitude
sys.configParams.mainOpeningListener = DRSS.core.dynamics.events.Altitude() ...
  .setEnabledOnInit(false) ...
  .setAlitude(sys.configParams.mainOpeningAltitude) ...
  .trigger(sys.configParams.main) ... % Main parachute comes into full effect
  .trigger(sys.configParams.drogue); % Drogue parachute comes back into effect

% Disable Drogue Dynamics Event: Disables drogue stage rocket drag upon trigger
sys.configParams.disableDrogueDynamics = DRSS.core.dynamics.events.TriggerOnEnable() ...
  .setDisableBoundDynamicsOnTrigger(true) ...
  .trigger(sys.configParams.drogue) ... % Before main shock cord is taught, the vehicle section experiences no force by the drogue
  .trigger(sys.configParams.rocketDrogueDescentDrag); % Disable drogue stage rocket drag

% Jettison Listener: Triggers nose cone jettison at a specified altitude
sys.configParams.jettisonListener = DRSS.core.dynamics.events.Altitude() ...
  .setEnabledOnInit(false) ...
  .setAlitude(sys.configParams.jettisonAltitude) ...
  .trigger(sys.configParams.jettisonEvent);

% Set Up Event Triggers

% Apogee Listener Triggers
sys.configParams.apogeeListener ...
  .trigger(sys.configParams.constantThetadDynamics) ...    % Enable constant theta dynamics
  .trigger(sys.configParams.disableAscentDynamics) ...     % Disable ascent dynamics
  .trigger(sys.configParams.rocketDrogueDescentDrag) ...   % Enable drogue stage rocket drag
  .trigger(sys.configParams.drogue) ...                    % Deploy drogue parachute
  .trigger(sys.configParams.mainDeploymentListener);       % Enable main deployment listener

% Launch Rail Clears: Start apogee detection
sys.configParams.launchRail.trigger(sys.configParams.apogeeListener);

% Main Deployment Listener Triggers
sys.configParams.mainDeploymentListener ...
  .trigger(sys.configParams.rocketMainDescentDrag) ...     % Enable main stage rocket drag
  .trigger(sys.configParams.disableDrogueDynamics) ...     % Disable drogue dynamics
  .trigger(sys.configParams.mainOpeningListener) ...    % Enable main parachute opening
  .trigger(sys.configParams.jettisonListener);             % Enable jettison listener
