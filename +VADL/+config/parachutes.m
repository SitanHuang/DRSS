function sys = parachutes(sys)

uc = DRSS.util.unitConv;

sys.configParams.drogue = DRSS.core.dynamics.Parachute() ...
  .setDiameter(15 * uc.in_to_m) ...
  .setCD(1.5) ...
  .setN(8) ... (toroidal = 9, elliptical = 8)
  .setEnabledOnInit(false);
% .deploymentTimeDelay(0);

sys.configParams.main = DRSS.core.dynamics.Parachute() ...
  .setDiameter(6 * 12 * uc.in_to_m) ...
  .setCD(2.2) ...
  .setN(9) ... (toroidal = 9, elliptical = 8)
  .setEnabledOnInit(false);