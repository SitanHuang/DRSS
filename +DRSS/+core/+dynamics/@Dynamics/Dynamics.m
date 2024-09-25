classdef Dynamics < handle
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

    function [this, sys, sysState0] = resetTransientData(this, sys, sysState0)
      % RESETTRANSIENTDATA The callback function to run between simulations
      %   since the Dynamics obj is a handle and may be reused in different
      %   System objects or the same System object simulated multiple times

      this.setEnabled(this.enabledOnInit, sys, sysState0);
    end
    function [dyn, sys, massChanged]=step(dyn, sys, sysState)
      % STEP callback on every integration step run once for each Dynamics in
      %   a system per step (useful for prepping values used in other funcs)
      %   and before resolve functions are run
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
      %   the modification process is not order dependent & reproducible via the
      %   sysState obj. Setting a marker in the sys obj or this dynamic obj is
      %   allowed as as long as it tries to recognize something that happened in
      %   the past.

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
  end

  methods (Access=protected)
    function [this, sys, ss] = onEnable(this, sys, ss)
    end
  end
end