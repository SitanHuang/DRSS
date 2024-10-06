classdef Parachute < DRSS.core.dynamics.Dynamics
  properties
    diameter % parachute projected diameter [in]
    CD % PROJECTED parachute vertical drag coefficient, NOT NOMINAL %NOTE= Fruity-Chutes provides projected drag coefficient
    n % parachute opening time parameter (toroidal = 9, elliptical = 8)

    CD_side = 0.38699 % parachute side drag coefficient (from CFD)
    CL_side = 0.36465 % parachute side lift coefficient (from CFD)

    deploymentTimeDelay = 0; % deployment time offset after t_enabled [s]
  end

  properties (Transient, SetAccess=protected)
    t_enabled = inf % time when this Dynamics is enabled [s]

    t_deploy = inf % time at deployment start [s]

    t_complete = inf % time at deployment complete [s]

    t_fill = inf % parachute filling time [s]

    t_fill_measure_time = inf
  end

  methods
    function this = setDiameter(this, val)
      this.diameter = val;
    end
    function this = setCD(this, val)
      this.CD = val;
    end
    function this = setCD_side(this, val)
      this.CD_side = val;
    end
    function this = setCL_side(this, val)
      this.CL_side = val;
    end
    function this = setN(this, val)
      this.n = val;
    end
    function this = setDeploymentTimeDelay(this, val)
      this.deploymentTimeDelay = val;
    end
  end

  methods
    function [this, sys, terminate, xdd, ydd, tdd, mdot]=resolve(this, sys, ss)
      terminate = false;

      xdd = 0;
      ydd = 0;
      tdd = 0;
      mdot = 0;

      tElapsed = ss.t - this.t_deploy;

      if tElapsed <= 0
        return;
      end

      if ss.t <= this.t_fill_measure_time
        this.t_fill_measure_time = ss.t;
        this.t_fill = this.n * this.diameter / abs(sqrt(ss.yd^2 + ss.xd^2));
        this.t_complete = this.t_deploy + this.t_fill;
      end

      [CD, CD_side, CL_side, S, S_side, ~] = DRSS.legacy.parachute_state( ...
        tElapsed, this.t_fill, ...
        this.diameter, ...
        this.CD, this.CD_side, this.CL_side, ...
        'projected', 'f');

      rho = ss.airDensity;

      v_inf_x = -(ss.windSpeed) - ss.xd;
      v_inf_y = -ss.yd;

      % aerodynamic forces
      FD = 0.5 * rho * CD * S * v_inf_y^2;
      FL = 0.5 * rho * CL_side * S_side * v_inf_x^2;
      FD_side = 0.5 * rho * CD_side * S_side * v_inf_x^2;

      % accelerations
      xdd = (FD_side) / ss.m;
      if v_inf_x < 0
        xdd = -xdd;
      end
      ydd = (FD + FL) / ss.m;
    end
  end

  methods (Access=protected)
    function [this, sys, ss] = onEnable(this, sys, ss)
      if ss.t > this.t_complete
        return;
      end

      this.t_enabled = min(this.t_enabled, ss.t);
      this.t_deploy = min(this.t_deploy, this.t_enabled + this.deploymentTimeDelay);
    end
  end
end