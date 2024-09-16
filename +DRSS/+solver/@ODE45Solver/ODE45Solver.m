classdef ODE45Solver < DRSS.solver.Solver
  % ODE45SOLVER A Runge-Kutta solver for a given System object.

  % ODE45 Configs
  properties
    % Simulation time span [t_start, t_end]
    timeSpan = [0, 300]

    % Options for MATLAB's ODE 45 solver
    %   From experimentation, RelTol is a MUCH more efficient option than
    %   MaxStep for increasing accuracy without incuring huge cost rise on
    %   computation. 1e-3 is sufficient for running batch simulations (i.e.,
    %   iterative rocket design optimization) while 1e-6 to 1e-8 is excellent
    %   for finalizing reports. Do not go to 1e-2; for example, motor burn is
    %   estimated to complete at 1.75s for a 2.0s burn time motor.
    %
    %   InitialStep needs to be small so to not entirely skip over the motor
    %   burn stage (MATLAB will choose dt=4s if this is not set); if the system
    %   isn't modeled from launch rail, the first few seconds of simulation are
    %   usually extremely sensitive. Setting InitialStep to a small amount saves
    %   computation as MATLAB doesn't need to iterate to decrease it further
    %
    %
    ODEOptions = odeset( ...
      'InitialStep', 0.001, ...
      'RelTol', 1e-3 ...
    )

    % Whether to capture non-integrated states (i.e., m, mdot, I), which slows down the solver drastically
    captureResultantParameters = false

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

    function this=setCaptureResultantParameters(this, val)
      this.captureResultantParameters = val;
    end
    function this=setPrintPerformanceSummary(this, val)
      this.performanceSummary = val;
    end

    function this=configureODE(this, varargin)
      this.ODEOptions = odeset(this.ODEOptions, varargin{:});
    end
  end

  % Solver Methods
  methods
    function [resultantStates, resultantParameters] = solve(this)
      tic;

      resultantParameters = DRSS.core.sim.SystemState();

      this.sys.recalcMassGroupProperties();

      this.ss = this.sys.systemState;
      this.ss.I = this.sys.momentOfInertia();
      this.ss.m = this.sys.m;

      [times, states] = ode45( ...
        @(t, y) this.integrationStep(t, y, this.sys, resultantParameters), ...
        this.timeSpan, ...
        this.ss.toStateVec(), ...
        odeset( ...
          this.ODEOptions, ...
          'Events', ...
          @(t, y) this.eventsStep(t, y, this.sys)) ...
      );

      simulationTime = toc;

      resultantStates = DRSS.core.sim.SystemState.fromSolvedStateVec(times, states);

      % Remove non-monotonic entries in resultantParameters (i.e., m & I)
      resultantParameters = this.cleanupResultantParameters(resultantParameters);

      if this.performanceSummary
        this.printPerformanceSummary(simulationTime, times, states, resultantParameters);
      end
    end
  end

  % Quasi-private methods (direct usage discouraged)
  methods
    dsdt = integrationStep(this, t, y, s, resultantParameters);
    [value, isterminal, direction] = eventsStep(this, t, y, s);
  end

  methods (Access=private)
    function resultantParameters=cleanupResultantParameters(this, resultantParameters)
      % CLEANUPRESULTANTPARAMETERS Removes non-monotonic entries in the time array
      %
      %   Modifies instances of the systemState by checking the time array `t`.
      %   For any time value that is less than its predecessor, it deletes all
      %   preceding entries from all properties.

      if length(resultantParameters.t) <= 1
        return;
      end

      keepIndices = true(size(resultantParameters.t));

      for i = 2:length(resultantParameters.t)
        keepIndices(1:i-1) = keepIndices(1:i-1) & (resultantParameters.t(1:i-1) <= resultantParameters.t(i));
      end

      propertyNames = fieldnames(resultantParameters);

      for i = 1:length(propertyNames)
        propName = propertyNames{i};
        if isnumeric(resultantParameters.(propName)) && length(resultantParameters.(propName)) == length(resultantParameters.t)
          resultantParameters.(propName) = resultantParameters.(propName)(keepIndices);
        end
      end
    end

    function printPerformanceSummary(this, simulationTime, times, states, resultantParameters)
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
      fprintf('  Memory Size of Parameters: %.3f MiB\n', resultantParameters.estimateMemorySize() / 1024 / 1024);
    end
  end
end