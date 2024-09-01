classdef Dynamics < handle
  properties
  end

  methods
    function this = resetTransientData(this)
    end
    % function dyn=step(dyn, sys)
    %   % STEP callback on every integration step run once for each Dynamics in
    %   %   a system per step (useful for prepping values used in other funcs)
    %   %
    %   %  The System passed via the sys argument contains systemState up to but
    %   %  does not include the current step.

    %   % Stub
    % end

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