classdef Solver < handle
  properties (Transient)
    sys
    ss = DRSS.core.sim.SystemState.createZeroState() % Initial SystemState for the system
  end

  % Abstraction Methods
  methods
    function this=Solver(system)
      this.sys = system;
    end

    function this=setState(this, val)
      this.ss = val;
    end
  end

  % Solver Methods
  methods (Abstract)
    solve(this)
  end

  methods (Access=protected)
    function this=prepareDynamics(this)
      len=length(this.sys.dynamicsList);

      for i=1:len
        dynamics = this.sys.dynamicsList{i};
        dynamics.resetTransientData();
      end
    end

    function this=stepThroughDynamics(this)
      len=length(this.sys.dynamicsList);

      for i=1:len
        dynamics = this.sys.dynamicsList{i};
        dynamics.step(dyn, this.sys, this.ss);
      end
    end

    function [termFlag, xddSum, yddSum, tddSum, mdotSum]=resolveDynamics(this)
      len=length(this.sys.dynamicsList);

      termFlag = false;
      xddSum = 0;
      yddSum = 0;
      tddSum = 0;
      mdotSum = 0;

      for i=1:len
        dynamics = this.sys.dynamicsList{i};
        [~, ~, terminate, xdd, ydd, tdd, mdot] = dynamics.resolve(dyn, this.sys, this.ss);

        termFlag = terminate || termFlag;

        xddSum = xddSum + xdd;
        yddSum = yddSum + ydd;
        tddSum = tddSum + tdd;
        mdotSum = mdotSum + mdot;
      end
    end
  end
end