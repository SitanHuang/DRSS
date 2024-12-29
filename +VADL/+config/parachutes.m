function sys = parachutes(sys)

uc = DRSS.util.unitConv;

sys.configParams.drogue = DRSS.core.dynamics.Parachute() ...
  .setDiameter(18 * uc.in_to_m) ... % 12 is min before rocket drag takes over
  .setCD(1.5) ...
  .setN(8) ... (toroidal = 9, elliptical = 8)
  .setOpeningModel(DRSS.core.dynamics.ParachuteOpeningModel.EXPONENTIAL) ...
  .setEnabledOnInit(false) ...
  .setDeploymentTimeDelay(-inf); % instantly open

sys.configParams.main = DRSS.core.dynamics.Parachute() ...
  .setDiameter(8 * 12 * uc.in_to_m) ...
  .setCD(2.2) ...
  .setN(9) ... (toroidal = 9, elliptical = 8)
  .setOpeningModel(DRSS.core.dynamics.ParachuteOpeningModel.EXPONENTIAL) ...
  .setEnabledOnInit(false) ...
  .setDeploymentTimeDelay(-inf); % instantly open
