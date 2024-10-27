function sys = massGroups(sys)

uc = DRSS.util.unitConv;

rocketCylindricalInertialGeometry = struct( ...
  'Ri', sys.configParams.rocket_diameter / 2 - 0.085 * uc.in_to_m, ...
  'Ro', sys.configParams.rocket_diameter / 2 ...
);

sys.configParams.noseCone = DRSS.core.obj.Mass("Nose Cone") ...
  .setM(6.8 * uc.lbm_to_kg) ...
  .setLen((7.5 + 6) * uc.in_to_m) ...
  .setCGX((7.5 + 6) / 2 * uc.in_to_m);
sys.configParams.payloadBay = DRSS.core.obj.MassGroup("Payload Bay") ...
  .appendChild(DRSS.core.obj.Mass("Payload bay") ...
    .setM(10.05 * uc.lbm_to_kg) ...
    .setLen(21 * uc.in_to_m) ...
    .setCGX(5.8 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Ballast") ...
    .setM(0.00 * uc.lbm_to_kg) ...
    .setLen(0 * uc.in_to_m) ...
    .setCGX(0 * uc.in_to_m));

sys.configParams.recoveryBay = DRSS.core.obj.MassGroup("Recovery bay") ...
  .appendChild(DRSS.core.obj.Mass("Drogue bay") ...
    .setM(2.74 * uc.lbm_to_kg) ...
    .setLen(12 * uc.in_to_m) ...
    .setCGX(12 / 2 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Avionics") ...
      .setM(4.35 * uc.lbm_to_kg) ...
      .setLen(2.5 * uc.in_to_m) ...
      .setCGX(2.5 / 2 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Main bay") ...
      .setM(3.87 * uc.lbm_to_kg) ...
      .setLen(14 * uc.in_to_m) ...
      .setCGX(14 / 2 * uc.in_to_m));


% sys.configParams.ballast = DRSS.core.obj.Mass("Ballast") ...
%   .setM(2 * uc.lbm_to_kg) ...
%   .setLen(0.1 * uc.in_to_m) ...
%   .setCGX(0.05 * uc.in_to_m);
sys.configParams.tail = DRSS.core.obj.MassGroup("Tail") ...
  .appendChild(DRSS.core.obj.Mass("Tail Tube") ...
    .setM((5.03 + 0.900) * uc.lbm_to_kg) ...
    .setLen(23 * uc.in_to_m) ...
    .setCGX(22 * uc.in_to_m)) ...
  .appendChild(DRSS.core.obj.Mass("Boat Tail") ...
      .setM(8 * uc.oz_to_kg) ...
      .setLen(6 * uc.in_to_m) ...
      .setCGX(2.9 * uc.in_to_m));

% Fictitious geometry just to make the CG work:
% sys.configParams.noseCone = DRSS.core.obj.Mass("Nose Cone") ...
%   .setM(5.1 * uc.lbm_to_kg) ...
%   .setLen(8.75 * uc.in_to_m) ...
%   .setCGX(10.3300 * uc.in_to_m);
% sys.configParams.payloadBay = DRSS.core.obj.Mass("Payload bay") ...
%   .setM(5.7685 * uc.lbm_to_kg) ...
%   .setLen(36 * uc.in_to_m) ...
%   .setCGX(22.6789 * uc.in_to_m);
% sys.configParams.recoveryBay = DRSS.core.obj.Mass("Recovery bay") ...
%   .setM(8.5713 * uc.lbm_to_kg) ...
%   .setLen(28.5 * uc.in_to_m) ...
%   .setCGX(13.586 * uc.in_to_m);
% sys.configParams.tail = DRSS.core.obj.Mass("Tail") ...
%   .setM(2.431 * uc.lbm_to_kg) ...
%   .setLen(28 * uc.in_to_m) ...
%   .setCGX(9.0820 * uc.in_to_m);

sys = sys ...
  .appendChild(sys.configParams.noseCone) ...
  .appendChild(sys.configParams.payloadBay) ...
  .appendChild(sys.configParams.recoveryBay) ...
  ... .appendChild(sys.configParams.ballast) ...
  .appendChild(sys.configParams.tail) ...
  .setInertialGeometryRecursive(rocketCylindricalInertialGeometry);