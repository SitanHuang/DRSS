function sys = eventsSetup(sys)

sys.configParams.jettisonEvent = DRSS.core.dynamics.Jettison() ...
  ... Set nose cone mass to 0:
  .trigger(sys.configParams.noseCone);

sys.configParams.apogeeListener = DRSS.core.dynamics.events.Apogee() ...
  .setEnabledOnInit(false); % Don't let launch rail jitter trigger false alarm

sys.configParams.disableAscentDynamics = DRSS.core.dynamics.events.TriggerOnEnable() ...
  .setDisableBoundDynamicsOnTrigger(true) ...
  .trigger(sys.configParams.rocketDynamics) ... disable ascent aerodynamics
  .trigger(sys.configParams.apogeeListener); % disable apogee listener

sys.configParams.constantThetaDynamics = DRSS.core.dynamics.ConstantThetaDynamics() ...
  .setEnabledOnInit(false) ...
  .setSpringConstant(1) ...
  .setDampingConstant(2) ...
  .setTargetAngleDeg(90);

sys.configParams.apogeeListener ...
  .trigger(sys.configParams.constantThetaDynamics) ...
  .trigger(sys.configParams.disableAscentDynamics) ... disable ascent stuff on apogee
  .trigger(sys.configParams.rocketDrogueDescentDrag) ... enable drogue stage rocket drag
  .trigger(sys.configParams.drogue); % deploy drogue parachute

% Only start apogee detection when launch rail clears:
sys.configParams.launchRail.trigger(sys.configParams.apogeeListener);

sys.configParams.mainDeploymentListener = DRSS.core.dynamics.events.Altitude() ...
  .setAlitude(sys.configParams.deployMainAltitude) ...
  .setEnabledOnInit(false) ...
  .trigger(sys.configParams.main) ... deploy main parachute
  .trigger(sys.configParams.rocketMainDescentDrag); % enable main stage rocket drag

sys.configParams.apogeeListener.trigger(sys.configParams.mainDeploymentListener);

sys.configParams.disableDrogueDynamics = DRSS.core.dynamics.events.TriggerOnEnable() ...
  .setDisableBoundDynamicsOnTrigger(true) ...
  .trigger(sys.configParams.rocketDrogueDescentDrag); % disable drogue stage rocket drag

sys.configParams.mainDeploymentListener.trigger(sys.configParams.disableDrogueDynamics);

sys.configParams.jettisonListener = DRSS.core.dynamics.events.Altitude() ...
  .setAlitude(sys.configParams.jettisonAltitude) ...
  .setEnabledOnInit(false) ...
  .trigger(sys.configParams.jettisonEvent);

sys.configParams.mainDeploymentListener.trigger(sys.configParams.jettisonListener);