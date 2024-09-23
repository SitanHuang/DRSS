classdef Jettison < DRSS.core.dynamics.Dynamics
  properties
    boundMass
  end

  methods
    function this = Jettison
      this.setEnabledOnInit(false);
    end
    function this = bindTo(this, val)
      this.boundMass = val;
    end
  end

  methods (Access=protected)
    function [this, sys, ss] = onEnable(this, sys, ss)
      this.boundMass.m = 0;
    end
  end
end