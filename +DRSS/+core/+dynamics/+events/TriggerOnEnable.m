classdef TriggerOnEnable < DRSS.core.dynamics.EventedDynamics
  % TRIGGERONENABLE An EventedDynamics obj to trigger the list of bounded dynamics on
  %   enable.

  methods
    function this = TriggerOnEnable()
      this.setEnabledOnInit(false);
    end
  end

  methods (Access=protected)
    function occurred = evaluateEvent(this, ss)
      occurred = true;
    end
  end
end
