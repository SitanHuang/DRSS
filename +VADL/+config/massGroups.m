function sys = massGroups(sys)

uc = DRSS.util.unitConv;

rocketCylindricalInertialGeometry = struct( ...
  'Ri', sys.configParams.rocket_diameter / 2 - 0.085 * uc.in_to_m, ...
  'Ro', sys.configParams.rocket_diameter / 2 ...
);

sys.configParams.noseCone = DRSS.core.obj.Mass("Nose Cone") ...
  .setM(1.28 * uc.lbm_to_kg) ...
  .setLen((9.75) * uc.in_to_m) ...
  .setCGX((9.75) * 3/4 * uc.in_to_m);

sys.configParams.jettisonedPayloadBayMass = DRSS.core.obj.Mass("Jettisoned Payload Bay Mass") ...
    ... .setM(6.77 * uc.lbm_to_kg) ... CDR
    ... .setM(7.2 * uc.lbm_to_kg) ... pre-shave, as built
    ... .setM(6.97 * uc.lbm_to_kg) ... planned shave
    ... .setM(6.62 * uc.lbm_to_kg) ... planned shave 2
    ... .setM(6.69 * uc.lbm_to_kg) ... post shave 2/18
    .setM(5.32 * uc.lbm_to_kg) ... new airframe, FRR MPP 3/11
    .setLen(10.75 * uc.in_to_m) ...
    .setCGX(10.75 * 0.3 * uc.in_to_m);

ballastMass = 2.3;

if isfield(sys.configParams, 'noBallast') && sys.configParams.noBallast
  ballastMass = 0;
end

if isfield(sys.configParams, 'ballastOffset') && sys.configParams.ballastOffset ~= 0
  ballastMass = ballastMass + sys.configParams.ballastOffset;
end

sys.configParams.payloadBay = DRSS.core.obj.MassGroup("Payload Bay") ...
  .appendChild(sys.configParams.jettisonedPayloadBayMass) ...
  .appendChild(DRSS.core.obj.Mass("Non-jettisoned Payload Bay Mass") ...
    .setM(6.26 * uc.lbm_to_kg) ...
    .setLen(25 * uc.in_to_m) ...
    .setCGX(25 * 0.86 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Payload Bay Ballast") ...
    .setM(ballastMass * uc.lbm_to_kg) ...
    .setLen(0) ...
    .setCGX(0));

sys.configParams.recoveryBay = DRSS.core.obj.MassGroup("Recovery bay") ...
  .appendChild(DRSS.core.obj.Mass("Drogue bay") ...
    .setM(4.12 * uc.lbm_to_kg) ...
    .setLen(6.5 * uc.in_to_m) ...
    .setCGX(6.5 / 2 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Avionics") ...
      .setM(3.59 * uc.lbm_to_kg) ...
      .setLen(2.5 * uc.in_to_m) ...
      .setCGX(2.5 / 2 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Main bay") ...
      .setM(4.81 * uc.lbm_to_kg) ...
      .setLen(6.5 * uc.in_to_m) ...
      .setCGX(6.5 / 2 * uc.in_to_m));


sys.configParams.tail = DRSS.core.obj.MassGroup("Tail") ...
  .appendChild(DRSS.core.obj.Mass("Tail Tube") ...
    ... .setM(6.23 * uc.lbm_to_kg) ...
    ... .setM(9.05 * uc.lbm_to_kg) ... ... pre-shave, as built
    ... .setM(6.60 * uc.lbm_to_kg) ... planned shave
    ... .setM(6.00 * uc.lbm_to_kg) ... planned shave 2
    .setM(6.95 * uc.lbm_to_kg) ... post shave 2/18
    .setLen(29 * uc.in_to_m) ...
    .setCGX(10 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Boat Tail") ...
      .setM(0 * uc.lbm_to_kg) ...
      .setLen(6 * uc.in_to_m) ...
      .setCGX(2.9 * uc.in_to_m));

sys = sys ...
  .appendChild(sys.configParams.noseCone) ...
  .appendChild(sys.configParams.payloadBay) ...
  .appendChild(sys.configParams.recoveryBay) ...
  .appendChild(sys.configParams.tail) ...
  .setInertialGeometryRecursive(rocketCylindricalInertialGeometry);

if isfield(sys.configParams, 'applyMassOffsetAtNoMotorCG') && sys.configParams.applyMassOffsetAtNoMotorCG
  sys.appendChild(DRSS.core.obj.Mass("Extra Mass Applied at CG of No Motor Config.") ...
      .setM(sys.configParams.applyMassOffsetAtNoMotorCG * uc.lbm_to_kg) ...
      .setLen(0) ...
      .setOffset(-sys.len + sys.cgX) ...
      .setCGX(0) ...
      .setInertialGeometry(struct( ...
        'Ri', sys.configParams.rocket_diameter / 2 - 0.085 * uc.in_to_m, ...
        'Ro', sys.configParams.rocket_diameter / 2 ...
      )));
end

sys.configParams.toBeJettisonedMasses = {
  sys.configParams.noseCone, ...
  sys.configParams.jettisonedPayloadBayMass
};

sys.configParams.jettisonedTotalMass = sys.configParams.noseCone.m + sys.configParams.jettisonedPayloadBayMass.m;