function sys = massGroups(sys)

uc = DRSS.util.unitConv;

rocketCylindricalInertialGeometry = struct( ...
  'Ri', sys.configParams.rocket_diameter / 2 - 0.085 * uc.in_to_m, ...
  'Ro', sys.configParams.rocket_diameter / 2 ...
);

sys.configParams.noseCone = DRSS.core.obj.Mass("Nose Cone") ...
  .setM(1.27 * uc.lbm_to_kg) ...
  .setLen((9.525) * uc.in_to_m) ...
  .setCGX((9.525) * 3/4 * uc.in_to_m);

sys.configParams.jettisonedPayloadBayMass = DRSS.core.obj.Mass("Jettisoned Payload Bay Mass") ...
    ... .setM(6.77 * uc.lbm_to_kg) ... CDR
    ... .setM(7.2 * uc.lbm_to_kg) ... pre-shave, as built
    ... .setM(6.97 * uc.lbm_to_kg) ... planned shave
    .setM(6.62 * uc.lbm_to_kg) ... planned shave 2
    .setLen(10.65 * uc.in_to_m) ...
    .setCGX(10.65 / 2 * uc.in_to_m);

sys.configParams.payloadBay = DRSS.core.obj.MassGroup("Payload Bay") ...
  .appendChild(sys.configParams.jettisonedPayloadBayMass) ...
  .appendChild(DRSS.core.obj.Mass("Non-jettisoned Payload Bay Mass") ...
    .setM(6.56 * uc.lbm_to_kg) ...
    .setLen(30.45 * uc.in_to_m) ...
    .setCGX(30.45 / 2 * uc.in_to_m));

sys.configParams.recoveryBay = DRSS.core.obj.MassGroup("Recovery bay") ...
  .appendChild(DRSS.core.obj.Mass("Drogue bay") ...
    .setM(3.84 * uc.lbm_to_kg) ...
    .setLen(7.23 * uc.in_to_m) ...
    .setCGX(7.23 / 2 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Avionics") ...
      .setM(3.39 * uc.lbm_to_kg) ...
      .setLen(2.74 * uc.in_to_m) ...
      .setCGX(2.74 / 2 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Main bay") ...
      .setM(5.84 * uc.lbm_to_kg) ...
      .setLen(7 * uc.in_to_m) ...
      .setCGX(7 / 2 * uc.in_to_m));


sys.configParams.tail = DRSS.core.obj.MassGroup("Tail") ...
  .appendChild(DRSS.core.obj.Mass("Tail Tube") ...
    ... .setM(6.23 * uc.lbm_to_kg) ...
    ... .setM(9.05 * uc.lbm_to_kg) ... ... pre-shave, as built
    ... .setM(6.60 * uc.lbm_to_kg) ... planned shave
    .setM(6.00 * uc.lbm_to_kg) ... planned shave 2
    .setLen(29 * uc.in_to_m) ...
    .setCGX(16 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Boat Tail") ...
      .setM(0.25 * uc.lbm_to_kg) ...
      .setLen(6 * uc.in_to_m) ...
      .setCGX(2.9 * uc.in_to_m));

sys = sys ...
  .appendChild(sys.configParams.noseCone) ...
  .appendChild(sys.configParams.payloadBay) ...
  .appendChild(sys.configParams.recoveryBay) ...
  .appendChild(sys.configParams.tail) ...
  .setInertialGeometryRecursive(rocketCylindricalInertialGeometry);

sys.configParams.toBeJettisonedMasses = {
  sys.configParams.noseCone, ...
  sys.configParams.jettisonedPayloadBayMass
};