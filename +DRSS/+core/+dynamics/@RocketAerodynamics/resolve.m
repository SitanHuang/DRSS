function [this, sys, terminate, xdd, ydd, tdd, mdot]=resolve(this, sys, ss)
  terminate = false;
  xdd = 0;
  ydd = 0;
  tdd = 0;
  mdot = 0;

  meta = this.aerodynamicProfile;

  L = sys.len;
  T0 = sys.launchSiteTemp + 273.15; % Kelvin
  p0 = sys.launchSitePressure;
  eps = sys.launchSiteElevation;

  B = 6.5e-3; % temperature lapse rate in troposphere [K/m]
  R = 287; % ideal gas constant for air [J/(kg*K)]
  gamma = 1.4; % spec. heat ratio for air
  g0 = 9.80665; % gravitational acceleration at mean sea level [m/s^2]

  % stability margin
  ssm = (meta.CP - sys.cgX) / meta.D;

  % center of pressure velocities
  v_CP_x = ss.xd + meta.D * ss.thetad * cos(ss.theta) * ssm;
  v_CP_y = ss.yd - meta.D * ss.thetad * sin(ss.theta) * ssm;

  % flow conditions
  v_wind = this.wind_calc(meta.Wr, eps, ss.y, 7); % 7 = power law denominator for wind

  phi = atan2(v_CP_x + v_wind, v_CP_y);
  v_inf = sqrt((v_CP_x + v_wind)^2 + v_CP_y^2);
  v = sqrt(ss.xd^2 + ss.yd^2);
  alpha = phi - ss.theta;
  [rho, T, p, mu] = this.atmosphere(ss.y, T0, p0, R, B, g0);
  Re = rho * v_inf * L / mu;
  M_inf = v_inf / sqrt(gamma * R * T);

  % aerodynamic coefficients
  [CA, CN, CD, CL, ~] = this.aerodynamics( ...
    Re, ...
    v, ...
    alpha, ...
    meta.cm, ...
    meta.D, ...
    L, ...
    meta.L_nose, ...
    this.L_body, ...
    this.A_planform, ...
    meta.L_tail, ...
    meta.L_tail_c, ...
    meta.L_tail_f, ...
    meta.D_tail, ...
    meta.A_fin, ...
    meta.N_fins, ...
    meta.A_fin_e, ...
    meta.L_fin, ...
    meta.t_fin, ...
    meta.s, ...
    meta.cr, ...
    meta.ct, ...
    M_inf, ...
    this.A, ...
    true, ...
    meta.CDr);

  % LEGACY CODE:
  % if z(ii) > z_aac && deploy_foil && ~are_foils_deployed
  %   i_aac = ii;
  %   are_foils_deployed = true;
  % end

  % increase tilt of airfoils
  % if are_foils_deployed && t(ii) < t(i_aac) + t_foil_deployment
  %   tilt_foil(ii) = delta_foil/t_foil_deployment * (t(ii) - t(i_aac));
  % end

  % % aerodynamic coefficients of airfoils
  % [CD_foil(ii), CL_foil(ii)] = NACA_coeff(airfoil_code,Re(ii),alpha(ii) + tilt_foil(ii),delta_foil);
  % CDi_foil(ii) = CL_foil(ii)^2/(pi*AR_foil*e_foil);

  % aerodynamic forces
  % F_lift_foil = 0.5 * 2 * rho * v_inf^2 * CL_foil * A_fin_foil;
  % F_drag_foil = 0.5 * 4 * rho * v_inf^2 * CD_foil * A_fin_foil;
  % FA_foil = F_drag_foil * cos(alpha) - F_lift_foil * sin(alpha);
  % FN_foil = F_lift_foil * cos(alpha) + F_drag_foil * sin(alpha);
  FA = 0.5 * CA * this.A * rho * v_inf^2; % + FA_foil;
  FN = 0.5 * CN * this.A * rho * v_inf^2; % + FN_foil;

  if ss.forceConstantTheta
    FN = 0;
  end

  % M_foil = FN_foil * L_foil_arm;
  M = FN * meta.D * ssm;

  m = ss.m;
  I = ss.I;

  xdd = - (FN / m) * cos(ss.theta) - ((FA / m) * sin(ss.theta));
  ydd = (FN / m) * sin(ss.theta) - ((FA / m) * cos(ss.theta));
  tdd = M / I;
end