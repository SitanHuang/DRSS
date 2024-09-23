classdef Apogee < DRSS.core.dynamics.EventedDynamics
  % APOGEE An EventedDynamics obj to trigger the list of bounded dynamics on
  %   apogee, and subsequently sets the system to constant theta. To avoid
  %   jitter on the launch rail triggering a false apogee event, use a
  %   LaunchRail Dynamics obj to trigger this Apogee event.

  methods
    function [this, sys, ss0] = resetTransientData(this, sys, ss0)
      resetTransientData@DRSS.core.dynamics.EventedDynamics(this, sys, ss0);
    end
  end

  methods (Access=protected)
    function occurred = evaluateEvent(this, ss)
      occurred = ss.yd < 0;
    end

    function [this, sys, ss] = onEventTrigger(this, sys, ss)
      [this, sys, ss] = onEventTrigger@DRSS.core.dynamics.EventedDynamics(this, sys, ss);
    end
  end
end
