function sys = massGroups(sys)

uc = DRSS.util.unitConv;

rocketCylindricalInertialGeometry = struct( ...
  'Ri', sys.configParams.rocket_diameter / 2 - 0.25 * uc.in_to_m, ...
  'Ro', sys.configParams.rocket_diameter / 2 ...
);

sys.configParams.noseCone = DRSS.core.obj.Mass("Nose Cone") ...
  .setM(5.1 * uc.lbm_to_kg) ...
  .setLen(7.5 * uc.in_to_m) ...
  .setCGX(7.5 / 2 * uc.in_to_m);
sys.configParams.payloadBay = DRSS.core.obj.Mass("Payload bay") ...
  .setM(5.7685 * uc.lbm_to_kg) ...
  .setLen(14.5 * uc.in_to_m) ...
  .setCGX(14.5 / 2 * uc.in_to_m);
sys.configParams.recoveryBay = DRSS.core.obj.Mass("Recovery bay") ...
  .setM(8.5713 * uc.lbm_to_kg) ...
  .setLen(28.5 * uc.in_to_m) ...
  .setCGX(13.586 * uc.in_to_m);
% sys.configParams.ballast = DRSS.core.obj.Mass("Ballast") ...
%   .setM(2 * uc.lbm_to_kg) ...
%   .setLen(0.1 * uc.in_to_m) ...
%   .setCGX(0.05 * uc.in_to_m);
sys.configParams.tail = DRSS.core.obj.Mass("Tail") ...
  .setM(2.431 * uc.lbm_to_kg) ...
  .setLen(27 * uc.in_to_m) ...
  .setCGX(9.0820 * uc.in_to_m);

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