classdef IEventTriggerDynamics < DRSS.core.dynamics.Dynamics
  properties
    % Dynamics to enable/disable
    boundDynamics = [];
  end

  properties (SetAccess=protected)
    % Time when the event is triggered
    t_trigger = inf;
    % System state at the time of trigger
    systemStateAtTrigger;

    % Whether to disable or enable the list of bound Dynamics on trigger
    disableBoundDynamicsOnTrigger = false;
  end

  methods
    function this = bindTo(this, dynamicsObj)
      % BINDTO Bind Dynamics that will be enabled or disabled upon event
      %   trigger. Note that this dynamicsObj can also be another
      %   IEventTriggerDynamics.
      this.boundDynamics = [this.boundDynamics {dynamicsObj}];
    end

    function this = setDisableBoundDynamicsOnTrigger(this, val)
      % SETDISABLEBOUNDDYNAMICSONTRIGGER to disable or enable the list of bound
      %   Dynamics on trigger.
      this.disableBoundDynamicsOnTrigger = val;
    end
  end

  methods
    function [this, sys, ss0] = resetTransientData(this, sys, ss0)
      [this, sys, ss0] = resetTransientData@DRSS.core.dynamics.Dynamics(this, sys, ss0);

      this.t_trigger = inf;
      this.systemStateAtTrigger = [];
    end

    function [this, sys, massChanged]=step(this, sys, ss)
      % STEP Check if the event condition is met and handle event trigger
      massChanged = false;

      if this.evaluateEvent(ss)
        if ss.t < this.t_trigger
          this.t_trigger = ss.t;
          this.systemStateAtTrigger = ss;
          this.onEventTrigger(sys, ss);
        end
      end
    end
  end

  methods (Abstract, Access=protected)
    % To be implemented by subclasses to define the event condition
    occurred = evaluateEvent(this, ss);
  end

  methods (Access = protected)
    function [this, sys, ss] = onEventTrigger(this, sys, ss)
      % ONEVENTTRIGGER Actions to perform when the event is triggered
      % Enable/disable bound dynamics
      for i = 1:length(this.boundDynamics)
        dyn = this.boundDynamics{i};
        dyn.setEnabled(~this.disableBoundDynamicsOnTrigger, sys, ss);
      end

      this.systemStateAtTrigger = ss.makeShallowCopy();
    end
  end
end