classdef Parachute < DRSS.core.dynamics.Dynamics
  properties
    diameter % parachute projected diameter [in]
    CD % PROJECTED parachute vertical drag coefficient, NOT NOMINAL %NOTE= Fruity-Chutes provides projected drag coefficient
    n % parachute opening time parameter (toroidal = 9, elliptical = 8)

    CD_side = 0.38699 % parachute side drag coefficient (from CFD)
    CL_side = 0.36465 % parachute side lift coefficient (from CFD)

    deploymentTimeDelay = 0; % deployment time offset after t_enabled [s]

    openingModel DRSS.core.dynamics.ParachuteOpeningModel = DRSS.core.dynamics.ParachuteOpeningModel.EXPONENTIAL;
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
    function this = setOpeningModel(this, val)
      this.openingModel = val;
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

      v_inf_x = -(ss.windSpeed) - ss.xd;
      v_inf_y = -ss.yd;

      if ss.t <= this.t_fill_measure_time
        this.t_fill_measure_time = ss.t;
        this.t_fill = this.n * this.diameter / sqrt(v_inf_y^2 + v_inf_x^2);
        this.t_complete = this.t_deploy + this.t_fill;
      end

      modelFlag = 'f';

      if this.openingModel == DRSS.core.dynamics.ParachuteOpeningModel.EXPONENTIAL
        modelFlag = 'e';
      elseif this.openingModel ~= DRSS.core.dynamics.ParachuteOpeningModel.FOURTH_ORDER
        error("Unknown parachute opening model selected: %s", this.openingModel);
      end

      [CD, CD_side, CL_side, S, S_side, ~] = DRSS.legacy.parachute_state( ...
        tElapsed, this.t_fill, ...
        this.diameter, ...
        this.CD, this.CD_side, this.CL_side, ...
        'projected', modelFlag); %#ok<PROPLC>
      % ^ Exponential model seems to match parachute jerk very well against real
      %   flight data; legacy codebase uses fourth-order (everyone was afraid to
      %   change or something) but that has been bullshit for many years against
      %   real data before me (Sitan Huang), and gives ridiculously low opening Gs

      rho = ss.airDensity;

      % % Fourth-order model:
      % t_f = 4 * this.t_fill;

      % Apf = pi * this.diameter^2 / 4;

      % x = min(tElapsed / t_f, 1);

      % CdpAp = Apf * (x^2 + (this.CD - 1)*x^4);

      % aerodynamic forces
      FD = 0.5 * rho * CD * S * v_inf_y^2;  %#ok<PROPLC>
      % FD = 0.5 * rho * CdpAp * v_inf_y^2;
      FL = 0.5 * rho * CL_side * S_side * v_inf_x^2;  %#ok<PROPLC>
      FD_side = 0.5 * rho * CD_side * S_side * v_inf_x^2;  %#ok<PROPLC>

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