function [this, sys, terminate, xdd, ydd, tdd, mdot]=resolve(this, sys, ss)
  terminate = false;
  xdd = 0;
  ydd = 0;
  tdd = 0;
  mdot = 0;

  meta = this.aerodynamicProfile;

  L = sys.len;

  R = 287; % ideal gas constant for air [J/(kg*K)]
  gamma = 1.4; % spec. heat ratio for air

  % stability margin
  ssm = (meta.CP - sys.cgX) / meta.D;

  this.ssm_min = min(this.ssm_min, ssm);
  this.ssm_max = max(this.ssm_max, ssm);

  % center of pressure velocities
  v_CP_x = ss.xd + meta.D * ss.thetad * cos(ss.theta) * ssm;
  v_CP_y = ss.yd - meta.D * ss.thetad * sin(ss.theta) * ssm;

  % flow conditions
  phi = atan2(v_CP_x + ss.windSpeed, v_CP_y);
  v_inf = sqrt((v_CP_x + ss.windSpeed)^2 + v_CP_y^2);
  v = sqrt(ss.xd^2 + ss.yd^2);
  alpha = phi - ss.theta;
  rho = ss.airDensity;
  T = ss.airTemp;
  p = ss.airPressure;
  mu = ss.airDynViscosity;
  Re = rho * v_inf * L / mu;
  M_inf = v_inf / sqrt(gamma * R * T);

  if M_inf < 0.1
    return;
  end

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
    meta.l_tail, ...
    meta.L_tail_c, ...
    meta.L_tail_f, ...
    meta.D_tail, ...
    meta.A_fin, ...
    meta.N_fins, ...
    meta.A_fin_e, ...
    meta.l_fin, ...
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