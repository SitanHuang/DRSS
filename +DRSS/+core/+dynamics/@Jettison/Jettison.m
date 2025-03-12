classdef Jettison < DRSS.core.dynamics.Dynamics
  properties
    % For backward compatibility: store the "last" mass triggered here.
    boundMass

    % List of all triggered masses
    boundMassList = {}
  end

  properties(Transient, SetAccess=protected)
    totalJettisonedMass = 0
  end

  properties(Transient, Access=protected)
    jettisoned = false;
  end

  methods
    function this = Jettison
      this.setEnabledOnInit(false);
    end

    function this = trigger(this, val)
      if ~iscell(val)
        val = {val};
      end

      this.boundMass = val{end};

      this.boundMassList = [this.boundMassList, val];
    end
  end

  methods (Access=protected)
    function [this, sys, ss] = onEnable(this, sys, ss)
      if ~this.jettisoned
        this.totalJettisonedMass = 0;
      end

      for i = 1:numel(this.boundMassList)
        massObj = this.boundMassList{i};

        if ~this.jettisoned
          this.totalJettisonedMass = this.totalJettisonedMass + massObj.m;
        end

        massObj.m = 0;
      end

      this.jettisoned = true;
      ss.massChanged = true;
    end
  end
end