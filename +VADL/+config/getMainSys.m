function sys = getMainSys(configParams)

if nargin < 1
  configParams = struct;
end

sys = DRSS.core.sim.System("Main Rocket");
sys.configParams = configParams;

VADL.config.gravity(sys);
VADL.config.geometry(sys);
VADL.config.massGroups(sys);

VADL.config.parachutes(sys);
VADL.config.descentDrag(sys);
VADL.config.dragPlate(sys);
VADL.config.launchCode(sys);
VADL.config.eventsSetup(sys);
VADL.config.buildFullRocketSystem(sys);