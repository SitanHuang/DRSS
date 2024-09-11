classdef ODE45Solver < DRSS.solver.Solver;
  % ODE45SOLVER A Runge-Kutta solver for a given System object.

  % ODE45 Configs
  properties
    TimeSpan = [0, 200] % Simulation time span [t_start, t_end]
    ODEOptions % Options for MATLAB's ODE 45 solver
  end

  % Abstraction Methods
  methods
    function this=setTimeSpan(this, val)
      this.TimeSpan = val;
    end
    function this=setODEOptions(this, val)
      this.ODEOptions = val;
    end

    function this=configureODE(this, varargin)
      this.ODEOptions = odeset(this.ODEOptions, varargin{:});
    end
  end

  % Solver Methods
  methods
    function solve(this)
      [times, states] = ode45( ...
        @(t, y) this.integrationStep(t, y, this.sys), ...
        this.timeSpan, ...
        this.ss.toStateVec(), ...
        this.ODEOptions ...
      );
    end
  end

  % Quasi-private methods (direct usage discouraged)
  methods
    dsdt = integrationStep(this, t, s);
  end
end