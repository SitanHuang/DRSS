classdef Dynamics < handle
  % DYNAMICS The superclass of all Dynamics objects that has the ability to
  %   modify the instantaneous SystemState of a System object as well as the
  %   System itself.
  %
  %   See DRSS.solver.MatlabODESolver.integrationStep for how Dynamics objects
  %     are evaluated during runtime.
  %
  %   See also: DRSS.core.sim.SystemState, DRSS.core.sim.System

  properties
    enabledOnInit = true;  % Flag to enable or disable the dynamics on init
  end

  properties (Transient, SetAccess=protected)
    enabled;        % Runtime flag to enable or disable dynamics

    t_lastEnable;
    t_lastDisable;
  end

  methods
    function this = setEnabledOnInit(this, val)
      this.enabledOnInit = val;
    end
  end

  methods
    function [this, sys, sysState0] = resetTransientData(this, sys, sysState0)
      % RESETTRANSIENTDATA The callback function to run between simulations
      %   since the Dynamics obj is a handle and may be reused in different
      %   System objects or the same System object simulated multiple times

      this.setEnabled(this.enabledOnInit, sys, sysState0);
    end
    function [dyn, sys, massChanged]=step(dyn, sys, sysState)
      % STEP callback on every integration step run once for each Dynamics in
      %   a system per step (useful for prepping values used in other funcs)
      %   and before any resolve functions are run
      %
      %  The System passed via the sys argument contains systemState up to but
      %  does not include the current step.
      %
      %  The mass in systemState will be recalculated after all step functions
      %  are run, and before any resolve functions are run

      % Stub
      massChanged = false;
    end

    function [dyn, sys, terminate, xdd, ydd, tdd, mdot]=resolve(dyn, sys, sysState)
      % RESOLVE The callback function to return some components of the forces
      %   and moments contributing to the total forces and moments at a
      %   particular timestep - specifically, components caused by this Dynamics
      %   object;
      %
      %   IMPORTANT!!!: The outputs must consider that the time in sysState may
      %   be non-monotonic. Modification of data in sys obj is allowed only if
      %   the modification process is not order dependent, and is reproducible
      %   via the sysState obj alone.

      terminate = false;
      xdd = 0;
      ydd = 0;
      tdd = 0;
      mdot = 0;
    end

    function [dyn, sys, terminate, xdd, ydd, tdd, mdot]=postAdjustment(dyn, sys, sysState)
      % POSTADJUSTMENT Callback function to run after all resolve functions are
      %   run. The sysState can be expected to be the current state after
      %   resolution of dynamics (but may include changes made by other
      %   postAdjustment callbacks).

      terminate = false;
      xdd = 0;
      ydd = 0;
      tdd = 0;
      mdot = 0;
    end


    function [this, val, sys, ss] = setEnabled(this, val, sys, ss)
      % Internal use only
      this.enabled = val;

      if val
        this.t_lastEnable = ss.t;
        this.onEnable(sys, ss);
      else
        this.t_lastDisable = ss.t;
      end
    end
  end

  methods (Access=protected)
    function [this, sys, ss] = onEnable(this, sys, ss)
    end
  end
end