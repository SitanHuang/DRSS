classdef EventReceivingDynamics < DRSS.core.dynamics.Dynamics
  methods (Abstract, Access=protected)
    % To be implemented by subclasses to define the event handler
    [this, sys, ss, triggerSource] = onEventReceive(this, sys, ss, triggerSource);
  end
end