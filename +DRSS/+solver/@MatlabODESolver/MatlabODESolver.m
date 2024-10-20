classdef MatlabODESolver < DRSS.solver.Solver
  % MatlabODESolver An ODE solver for a given System object using MATLAB's
  %   built-in ode suites.
  %
  %   Example:
  %       % Create an instance of the solver
  %       solver = DRSS.solver.MatlabODESolver();
  %
  %       % Configure the solver
  %       solver
  %         .setTimeSpan([0, 100]);
  %         .configureODE('RelTol', 1e-6, 'AbsTol', 1e-8);
  %         .setCaptureResultantParameters(true);
  %
  %       % Run the solver
  %       [states, timeVariantParams] = solver.solve();
  %
  %   See also: odeset, ode113, ode45, DRSS.core.sim.System

  % Configs
  properties
    % Simulation time span [t_start, t_end], seconds
    timeSpan = [0, 300]

    % ODEOPTIONS Options for MATLAB's ODE 45 solver
    %
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
    %   By using overrideODEFunc and setting the solver method to other than
    %   ode45/ode89, RelTol can be increased without performance / memory costs.
    %   Anecdotally, using ode113 at 1e-9 tolerance is great for both
    %   performance & accuracy.
    %
    ODEOptions = odeset( ...
      'InitialStep', 0.001, ...
      'RelTol', 1e-6, ...
      'MaxStep', 0.1 ...
    )

    % CAPTURERESULTANTPARAMETERS Flag to capture non-integrated, time variant parameters.
    %
    %   When set to true, the solver captures additional parameters during the
    %   simulation, such as mass, mass flow rate, and moment of inertia. Enabling
    %   this option may slow down the solver due to increased data storage.
    %
    %   Default: false
    captureResultantParameters = false

    % PERFORMANCESUMMARY Flag to print performance summary after solving.
    %
    %   Default: true
    performanceSummary = true

    % DEBUGFLAG Flag to enable debug mode for verbose, step-by-step integration output.
    debugFlag = false

    % Use ODE
    odeFunc=@ode113
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

    function this=overrideODEFunc(this, val)
      this.odeFunc = val;
    end
  end

  % Solver Methods
  methods
    function [resultantStates, resultantParameters] = solve(this)
      % SOLVE Run the ODE solver and return the resultant states and parameters.
      %
      %   [states, timeVariantParams] = solve(this) runs the ODE solver over the specified
      %   time span and returns the states and any captured parameters.
      %
      %   Outputs:
      %       resultantStates - An object of type DRSS.core.sim.SystemState
      %                         containing the state variables over time.
      %       resultantParameters - An object of type DRSS.core.sim.SystemState
      %                             containing the captured, time-variant
      %                             parameters over time. Note that this will
      %                             only be valid if solver.captureResultantParameters
      %                             is set to true.
      %
      %   Note:
      %       The solver uses the systemState associated with the system object
      %       (this.sys) as the initial condition.
      tic;

      resultantParameters = DRSS.core.sim.SystemState();

      this.sys.recalcMassGroupProperties();

      this.ss = this.sys.systemState;
      this.ss.I = this.sys.momentOfInertia();
      this.ss.m = this.sys.m;

      for i=1:length(this.sys.dynamicsList)
        dyn = this.sys.dynamicsList{i};
        dyn.resetTransientData(this.sys, this.ss);
      end

      [times, states] = this.odeFunc( ...
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

      resultantStates.xdd = gradient(resultantStates.xd, resultantStates.t);
      resultantStates.ydd = gradient(resultantStates.yd, resultantStates.t);
      resultantStates.thetadd = gradient(resultantStates.thetad, resultantStates.t);

      % Remove non-monotonic entries in resultantParameters (i.e., m & I)
      resultantParameters = this.cleanupResultantParameters(resultantParameters);

      if this.performanceSummary
        this.printPerformanceSummary(simulationTime, times, states, resultantParameters);
      end
    end
  end

  % Quasi-private methods (direct usage discouraged)
  methods
    % INTEGRATIONSTEP Compute the derivative of the state vector at time t.
    %
    %   dsdt = integrationStep(this, t, y, s, resultantParameters) computes
    %   the time derivative of the state vector y at time t using the system
    %   dynamics defined in the system object s.
    %
    %   Inputs:
    %       t - Current time.
    %       y - Current state vector.
    %       s - System object.
    %       resultantParameters - SystemState to store captured timeVariant parameters.
    %
    %   Output:
    %       dsdt - Time derivative of the state vector.
    %
    %   Note:
    %       This method is called internally by the ODE solver. It is encouraged
    %       to not be called directly by users.
    dsdt = integrationStep(this, t, y, s, resultantParameters);

    % EVENTSSTEP Event function for the ODE solver.
    %
    %   [value, isterminal, direction] = eventsStep(this, t, y, s) defines
    %   the events that the ODE solver should monitor during integration.
    %   Specifically, this event halts integration when systemState.terminate is
    %   set to true
    %
    %   Inputs:
    %       t - Current time.
    %       y - Current state vector.
    %       s - System object.
    %
    %   Outputs:
    %       value - Value(s) of the event function(s) (vector).
    %       isterminal - Boolean flag(s) indicating if integration should
    %                    terminate when event is detected (vector).
    %       direction - Direction(s) of zero-crossing to detect (+1, -1, 0).
    %
    %   Note:
    %       This method is called internally by the ODE solver. It is encouraged
    %       to not be called directly by users.
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

      fn = fieldnames(resultantParameters.params);
      for k = 1:numel(fn)
        val = resultantParameters.params.(fn{k});
        if isnumeric(val)
          resultantParameters.params.(fn{k}) = resultantParameters.params.(fn{k})(keepIndices);
        end
      end
    end

    function printPerformanceSummary(this, simulationTime, times, states, resultantParameters)
      % PRINTPERFORMANCESUMMARY Print a summary of the solver's performance.

      minTimeStep = min(diff(times));
      maxTimeStep = max(diff(times));
      framesTotal = length(times);
      approxMemorySize = whos('states');
      approxMemorySize = approxMemorySize.bytes;

      fprintf('# MatlabODESolver Performance Summary\n');
      fprintf('  Simulation Time: %.3f sec, %.1f ms per frame, %.1f sims per min\n', simulationTime, simulationTime / framesTotal * 1e3, 1 / simulationTime * 60);
      fprintf('  Time Step: %d s (min) - %.3f ms (max)\n', minTimeStep, maxTimeStep * 1e3);
      fprintf('  Time Span: %.3f s - %.3f s\n', min(times), max(times));
      fprintf('  Total Frames: %d\n', framesTotal);
      fprintf('  Memory Size of States: %.3f MiB\n', approxMemorySize / 1024 / 1024);
      fprintf('  Memory Size of Parameters: %.3f MiB\n', resultantParameters.estimateMemorySize() / 1024 / 1024);
    end
  end
end