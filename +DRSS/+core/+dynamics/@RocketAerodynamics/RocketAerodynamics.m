classdef RocketAerodynamics < DRSS.core.dynamics.Dynamics
  properties
    aerodynamicProfile
  end

  % For internal use:
  properties (Access=protected, Transient)
    A
    L_body
    A_body
    A_nose
    A_tail
    A_planform
  end

  methods
    function this = RocketAerodynamics(varargin)
      this = this.initialize(varargin);
    end

    [this, sys, sysState0] = resetTransientData(this, sys, sysState0);

    [this, sys, terminate, xdd, ydd, tdd, mdot] = resolve(this, sys, sysState);
  end

  methods (Access=private)
    this = initialize(this, inputs);
  end

  methods (Access=public, Static)
    % Legacy Functions:
    w = wind_calc(wr, zr, z, xi);
    [rho, T, p, mu] = atmosphere(z, T0, p0, R, B, g0);
    [CA, CN, CD, CL, CP] = aerodynamics(Re, v, alpha, cm, D, L, L_nose, L_body, A_planform, l_tail, L_tail_c, L_tail_f, D_tail, A_fin, n_fins, A_fin_e, l_fin, t_fin, s, cr, ct, M_inf, A, CD_override, CDr);
  end
end