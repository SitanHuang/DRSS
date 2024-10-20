classdef RocketAerodynamics < DRSS.core.dynamics.Dynamics
  properties
    aerodynamicProfile
  end

  % For internal use:
  properties (SetAccess=protected, Transient)
    A
    L_body
    A_body
    A_nose
    A_tail
    A_planform
    A_side

    ssm_min = inf
    ssm_max = -inf
    cdr_min = inf
    cdr_max = -inf
    cp_calc_min = inf
    cp_calc_max = -inf
  end

  methods
    function this = RocketAerodynamics(varargin)
      this = this.initialize(varargin);
    end

    function this = recalcTransientParameters(this, sys)
      meta = this.aerodynamicProfile;

      L = sys.len;

      this.A = (pi / 4) * meta.D ^ 2; % rocket cross section
      this.L_body = L - meta.L_nose - meta.L_tail_c - - meta.L_tail_f;
      this.A_body = this.L_body * meta.D; % body section projected side area [m^2]
      this.A_nose = (meta.L_nose * meta.D) / 2*1.2; % approximate nose cone projected side area (triangle) [m^2]
      this.A_tail = meta.L_tail_f * meta.D + (meta.L_tail_c * (meta.D + meta.D_tail)) / 2 + meta.L_tail_rr * meta.D_tail_rr; % boat tail projected side area [m^2]
      this.A_planform = this.A_body + this.A_nose + this.A_tail; % side projected area of rocket excluding fins [m^2]
      this.A_side = this.A_planform + meta.A_fin * 2;
      % A_fin_foil = meta.s_foil * (meta.cr_foil + meta.ct_foil) / 2;
    end

    [this, sys, sysState0] = resetTransientData(this, sys, sysState0);

    [this, sys, terminate, xdd, ydd, tdd, mdot] = resolve(this, sys, sysState);
  end

  methods (Access=private)
    this = initialize(this, inputs);
  end

  methods (Access=public, Static)
    % Legacy Functions:
    [CA, CN, CD, CL, CP] = aerodynamics(Re, v, alpha, cm, D, L, L_nose, L_body, A_planform, l_tail, L_tail_c, L_tail_f, D_tail, A_fin, n_fins, A_fin_e, l_fin, t_fin, s, cr, ct, M_inf, A, CD_override, CDr);
  end
end