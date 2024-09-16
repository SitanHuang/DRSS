classdef ODE45Solver < DRSS.solver.Solver
  % ODE45SOLVER A Runge-Kutta solver for a given System object.

  % ODE45 Configs
  properties
    timeSpan = [0, 200] % Simulation time span [t_start, t_end]
    ODEOptions = odeset('InitialStep', 0.01, 'MaxStep', 1) % Options for MATLAB's ODE 45 solver

    performanceSummary = true
    debugFlag = false
  end

  % Abstraction Methods
  methods
    function this=setTimeSpan(this, val)
      this.timeSpan = val;
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
    function [resultantStates] = solve(this)
      tic;

      this.ss = this.sys.systemState;

      [times, states] = ode45( ...
        @(t, y) this.integrationStep(t, y, this.sys), ...
        this.timeSpan, ...
        this.ss.toStateVec(), ...
        odeset( ...
          this.ODEOptions, ...
          'Events', ...
          @(t, y) this.eventsStep(t, y, this.sys)) ...
      );

      resultantStates = DRSS.core.sim.SystemState.fromSolvedStateVec(times, states);

      if this.performanceSummary
        this.printPerformanceSummary(times, states);
      end
    end
  end

  % Quasi-private methods (direct usage discouraged)
  methods
    dsdt = integrationStep(this, t, y, s);
    [value, isterminal, direction] = eventsStep(this, t, y, s);
  end

  methods (Access=private)
    function printPerformanceSummary(this, times, states)
      simulationTime = toc; %


      minTimeStep = min(diff(times));
      maxTimeStep = max(diff(times));
      framesTotal = length(times);
      approxMemorySize = whos('states');
      approxMemorySize = approxMemorySize.bytes;

      fprintf('# ODE45Solver Performance Summary\n');
      fprintf('  Simulation Time: %.3f sec, %.1f ms per frame, %.1f sims per min\n', simulationTime, simulationTime / framesTotal * 1e3, 1 / simulationTime * 60);
      fprintf('  Time Step: %d s (min) - %.3f ms (max)\n', minTimeStep, maxTimeStep * 1e3);
      fprintf('  Time Span: %.3f s - %.3f s\n', min(times), max(times));
      fprintf('  Total Frames: %d\n', framesTotal);
      fprintf('  Memory Size of States: %.3f MiB\n', approxMemorySize / 1024 / 1024);
    end
  end
end