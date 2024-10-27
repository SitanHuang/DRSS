function sys = descentDrag(sys)

rocketDynamics = sys.configParams.rocketDynamics;
rocketDynamics.recalcTransientParameters(sys);

% reference area for vertical drag calculations
Ar_main = (rocketDynamics.A + rocketDynamics.A_side) / 2; % take average of frontal and side projected areas
CDr_drogue = (1.42 * 1.41 * rocketDynamics.aerodynamicProfile.A_fin + 0.56 * (rocketDynamics.A_side)) / rocketDynamics.A;
% Ar_main = rocketDynamics.A * 0.75 + rocketDynamics.A_side * 0.25;
% CDr_drogue = 0.5;

sys.configParams.rocketDrogueDescentDrag = DRSS.core.dynamics.SimpleDrag() ...
  .setArea(rocketDynamics.A) ...
  .setCD(CDr_drogue) ...
  .setArea_side(rocketDynamics.A_side) ... 0.343 = legacy
  .setCD_side(1) ...
  .setEnabledOnInit(false);

sys.configParams.rocketMainDescentDrag = DRSS.core.dynamics.SimpleDrag() ...
  .setArea(Ar_main) ...
  .setCD(0.5) ...
  .setArea_side(rocketDynamics.A_side) ... 0.343 = legacy
  .setCD_side(1) ...
  .setEnabledOnInit(false);

% Legacy drogue:
% Ar = 0.1810
% A = 0.0190 (same as DRSS)
% Ar_side = 0.3430
% CDr = 11.9188
% A_fin = 0.0172
% L_nose=0.1905; L_tail=0.1524; L=1.9685; L_body = 1.6256
% A_body = 0.2529, A_nose=0.0178, A_tail=0.0225
% Ar_side = 0.3430
% FDr(ii) = 0.5*rho(ii)*CDr*A*v_inf_z(ii)^2;
% FDr_side(ii) = 0.5*rho(ii)*CDr_side*Ar_side*v_inf_x(ii)^2;
% Legacy Main:
% CDr = CDr_original;
% FDr(ii) = 0.5*rho(ii)*CDr*Ar*v_inf_z(ii)^2;
% FDr_side(ii) = 0.5*rho(ii)*CDr_side*Ar_side*v_inf_x(ii)^2;