function sys = getSubscaleSys(configParams)

if nargin < 1
  configParams = struct;
end

sys = DRSS.core.sim.System("Subscale Rocket");
sys.configParams = configParams;

VADL.config.subscale.gravity(sys);
VADL.config.subscale.geometry(sys);
VADL.config.subscale.massGroups(sys);

VADL.config.subscale.parachutes(sys);
VADL.config.subscale.descentDrag(sys);
VADL.config.subscale.launchCode(sys);
VADL.config.subscale.eventsSetup(sys);
VADL.config.subscale.buildFullRocketSystem(sys);