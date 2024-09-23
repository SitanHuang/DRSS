classdef Altitude < DRSS.core.dynamics.IEventTriggerDynamics
  % ALTITUDE An IEventTriggerDynamics obj to trigger the list of bounded
  %   dynamics on reaching above or below altitude.

  properties
    comparisonMethod DRSS.core.dynamics.events.Altitude.AltitudeComparisonMethod = DRSS.core.dynamics.events.Altitude.AltitudeComparisonMethod.BELOW_THRESHOLD
    alitude
  end

  methods
    function this = setComparisonMethod(this, val)
      this.comparisonMethod = val;
    end
    function this = setAlitude(this, val)
      this.alitude = val;
    end
  end

  methods (Access=protected)
    function occurred = evaluateEvent(this, ss)
      if this.comparisonMethod == DRSS.core.dynamics.events.Altitude.AltitudeComparisonMethod.BELOW_THRESHOLD
        occurred = ss.y < this.alitude;
      else
        occurred = ss.y > this.alitude;
      end
    end
  end
end
