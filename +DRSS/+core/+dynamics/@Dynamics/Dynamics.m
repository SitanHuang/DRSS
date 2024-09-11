classdef Dynamics < handle
  properties
  end

  methods
    function this = resetTransientData(this)
      % RESETTRANSIENTDATA The callback function to run between simulations
      %   since the Dynamics obj is a handle and may be reused in different
      %   System objects or the same System object simulated multiple times
    end
    function [dyn, sys]=step(dyn, sys, sysState)
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
    end

    function [dyn, sys, terminate, xdd, ydd, tdd, mdot]=resolve(dyn, sys, sysState)
      % RESOLVE The callback function to return some components of the forces
      %   and moments contributing to the total forces and moments at a
      %   particular timestep - specifically, components caused by this Dynamics
      %   object;
      %
      %   IMPORTANT!!!: The outputs must ONLY be produced using information in
      %   the current sysState. Modification of data in sys obj is allowed
      %   only if the modification process is not order dependent & reproducible
      %   via the sysState obj. Setting a marker in the sys obj or this dynamic
      %   obj is allowed as as long as it tries to recognize something that
      %   happened in the past, rather than something that is happening in the
      %   current tick
    end
  end
end