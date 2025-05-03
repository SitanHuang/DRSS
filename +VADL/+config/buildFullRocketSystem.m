function sys = buildFullRocketSystem(sys)

uc = DRSS.util.unitConv();

sys = sys ...
  .subjectTo(sys.configParams.gravityDynamics) ...
  ... Ascent-specific:
  .subjectTo(sys.configParams.launchRail) ...
  .subjectTo(sys.configParams.motorDynamics) ...
  .subjectTo(sys.configParams.rocketDynamics) ...
  ... Descent-specific:
  .subjectTo(sys.configParams.main) ...
  .subjectTo(sys.configParams.rocketDrogueDescentDrag) ...
  .subjectTo(sys.configParams.rocketMainDescentDrag) ...
  ... Special dynamics:
  .subjectTo(sys.configParams.jettisonEvent) ...
  .subjectTo(sys.configParams.constantThetadDynamics) ...
  ... Triggers:
  .subjectTo(sys.configParams.apogeeListener) ...
  .subjectTo(sys.configParams.disableAscentDynamics) ...
  .subjectTo(sys.configParams.mainDeploymentListener) ...
  .subjectTo(sys.configParams.mainOpeningListener) ...
  .subjectTo(sys.configParams.disableDrogueDynamics) ...
  .subjectTo(sys.configParams.jettisonListener);

if isfield(sys.configParams, 'disableJettison') && sys.configParams.disableJettison
  sys.configParams.jettisonListener.setAlitude(-inf);
end

if ~isfield(sys.configParams, 'disableDrogue') || ~sys.configParams.disableDrogue
  sys = sys.subjectTo(sys.configParams.drogue);
end

if isfield(sys.configParams, 'dragPlate')
  sys = sys.subjectTo(sys.configParams.dragPlate);
end

% Calculate geometry based on system length with massing now available:
sys.configParams.rocketDynamics.recalcTransientParameters(sys);

sys.configParams.ssm = (sys.configParams.rocketDynamics.aerodynamicProfile.CP - sys.cgX) / sys.configParams.rocketDynamics.aerodynamicProfile.D;

sys.configParams.sectionalMasses = [
  sys.configParams.payloadBay.m + sys.configParams.noseCone.m
  sys.configParams.recoveryBay.m
  sys.configParams.tail.m + sys.configParams.motorDynamics.motorMassGroup.m
] .* uc.kg_to_lbm;

if isfield(sys.configParams, 'noMotor') && sys.configParams.noMotor
  sys.configParams.sectionalMasses(end) = sys.configParams.sectionalMasses(end) - sys.configParams.motorDynamics.motorMassGroup.m .* uc.kg_to_lbm;
else
  sys.configParams.sectionalMasses(end) = sys.configParams.sectionalMasses(end) - sys.configParams.motorDynamics.motor_params.m_prop0 .* uc.kg_to_lbm;
end

% Jettison case:
if ~(isfield(sys.configParams, 'disableJettison') && sys.configParams.disableJettison)
  sys.configParams.sectionalMasses(1) = sys.configParams.sectionalMasses(1) - sys.configParams.jettisonedTotalMass .* uc.kg_to_lbm;
end