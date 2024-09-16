classdef Gravity < DRSS.core.dynamics.Dynamics
  properties (Constant)
    RE = 6.37e6 % Earth radius [m]
    g0 = 9.80665 % gravitational acceleration at mean sea level [m/s^2]
  end

  properties
    eps = 0 % launch site elevation above mean sea level [m]
    terminateOnGrounding = false
  end

  methods
    function this=setEPS(this, eps)
      this.eps = eps;
    end

    function this=setTerminateOnGrounding(this, tog)
      this.terminateOnGrounding = tog;
    end


    function [this, sys, terminate, xdd, ydd, tdd, mdot]=resolve(this, sys, ss)
      terminate = false;

      xdd = 0;
      ydd = 0;
      tdd = 0;
      mdot = 0;

      if abs(ss.y) < 1e-3 % tolerance for some finite penetration of ground
        return;
      end

      if ss.y < 0
        % Rocket below ground, it's OVER
        terminate = this.terminateOnGrounding;
        return;
      end

      ydd = -this.g0 * (this.RE / (this.RE + this.eps + ss.y))^2;
    end
  end
end