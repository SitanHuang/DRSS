classdef SimpleDrag < DRSS.core.dynamics.Dynamics
  properties
    area % [m^2]
    area_side % [m^2]
    CD
    CD_side
  end

  methods
    function this = setArea(this, val)
      this.area = val;
    end
    function this = setCD(this, val)
      this.CD = val;
    end
    function this = setArea_side(this, val)
      this.area_side = val;
    end
    function this = setCD_side(this, val)
      this.CD_side = val;
    end
  end

  methods
    function [this, sys, terminate, xdd, ydd, tdd, mdot]=resolve(this, sys, ss)
      terminate = false;

      xdd = 0;
      ydd = 0;
      tdd = 0;
      mdot = 0;
      rho = ss.airDensity;

      v_inf_x = -(ss.windSpeed) - ss.xd;
      v_inf_y = -ss.yd;

      % aerodynamic forces
      FD = 0.5 * rho * this.CD * this.area * v_inf_y^2;
      FD_side = 0.5 * rho * this.CD_side * this.area_side * v_inf_x^2;

      % accelerations
      xdd = FD_side / ss.m;
      if v_inf_x < 0
        xdd = -xdd;
      end
      ydd = FD / ss.m;
    end
  end
end