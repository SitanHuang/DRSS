function sys = eventsSetup(sys)

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

% Set Up Event Triggers

% Apogee Listener Triggers
sys.configParams.apogeeListener ...
  .trigger(sys.configParams.constantThetadDynamics) ...    % Enable constant theta dynamics
  .trigger(sys.configParams.disableAscentDynamics) ...     % Disable ascent dynamics
  .trigger(sys.configParams.rocketMainDescentDrag) ...     % Enable main stage rocket drag
  .trigger(sys.configParams.main);                         % Deploy main parachute

% Launch Rail Clears: Start apogee detection
sys.configParams.launchRail.trigger(sys.configParams.apogeeListener);