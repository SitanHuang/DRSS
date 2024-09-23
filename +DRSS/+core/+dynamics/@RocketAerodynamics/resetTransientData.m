function [this, sys, ss0] = resetTransientData(this, sys, ss0)
  [this, sys, ss0] = resetTransientData@DRSS.core.dynamics.Dynamics(this, sys, ss0);

  meta = this.aerodynamicProfile;

  L = sys.len;

  this.A = (pi / 4) * meta.D ^ 2; % rocket cross section
  this.L_body = L - meta.L_nose - meta.L_tail;
  this.A_body = this.L_body * meta.D; % body section projected side area [m^2]
  this.A_nose = (meta.L_nose * meta.D) / 2*1.2; % approximate nose cone projected side area (triangle) [m^2]
  this.A_tail = meta.L_tail_f * meta.D + (meta.L_tail_c * (meta.D + meta.D_tail)) / 2 + meta.L_tail_rr * meta.D_tail_rr; % boat tail projected side area [m^2]
  this.A_planform = this.A_body + this.A_nose + this.A_tail; % side projected area of rocket excluding fins [m^2]
  % A_fin_foil = meta.s_foil * (meta.cr_foil + meta.ct_foil) / 2;
end