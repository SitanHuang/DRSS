classdef Apogee < DRSS.core.dynamics.IEventTriggerDynamics
  % APOGEE An IEventTriggerDynamics obj to trigger the list of bounded dynamics on
  %   apogee, and subsequently sets the system to constant theta. To avoid
  %   jitter on the launch rail triggering a false apogee event, use a
  %   LaunchRail Dynamics obj to trigger this Apogee event.

  methods (Access=protected)
    function occurred = evaluateEvent(this, ss)
      occurred = ss.yd < 0;
    end
    function [this, sys, ss] = onEnable(this, sys, ss)
      ss.forceConstantTheta = true;
    end
  end
end
