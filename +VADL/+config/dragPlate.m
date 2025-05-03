function sys = dragPlate(sys)

uc = DRSS.util.unitConv;

if ~isfield(sys.configParams, 'dragPlateEnable')
  sys.configParams.dragPlateEnable = false;
end
if ~isfield(sys.configParams, 'dragPlateCDFunc')
  sys.configParams.dragPlateCDFunc = @(ss) 0.6;
end
if ~isfield(sys.configParams, 'dragPlateArea')
  sys.configParams.dragPlateArea = 0.05;
end
if ~isfield(sys.configParams, 'dragPlateFillTime')
  sys.configParams.dragPlateFillTime = 1; % s
end
if ~isfield(sys.configParams, 'dragPlateDelay')
  sys.configParams.dragPlateDelay = 0; % s after trigger
end
if ~isfield(sys.configParams, 'dragPlateTriggerFunc')
  % default: deploy once altitude > 3500 ft
  sys.configParams.dragPlateTriggerFunc = ...
      @(ss) ss.y > (3500 * uc.ft_to_m);
end

if sys.configParams.dragPlateEnable
  dp = DRSS.core.dynamics.SimpleDragPlate() ...
          .setCDFunc(sys.configParams.dragPlateCDFunc) ...
          .setArea(sys.configParams.dragPlateArea) ...
          .setFillTime(sys.configParams.dragPlateFillTime) ...
          .setTriggerFunc(sys.configParams.dragPlateTriggerFunc) ...
          .setDeploymentTimeDelay(sys.configParams.dragPlateDelay);
  sys.configParams.dragPlate = dp;
end