classdef LaunchRail < DRSS.core.dynamics.IEventTriggerDynamics
  % LAUNCHRAIL A Dynamics object that sets the initial state of the system and
  %   forces theta to be constant while a System is on the launch rail.
  %   LaunchRail implements the IEventTriggerDynamics abstract class to trigger
  %   other dynamics when rail exit is detected.
  %   LaunchRail also disables itself via setEnabled(false) on detecting
  %   systemState.t > t_launchRailCleared to avoid interfering with
  %   systemState.forceConstantTheta with other Dynamics.

  properties
    launchRailAngleDeg = 0 % launch angle [deg]
    launchRailLength % launch rail length [m]

    launchRailButtonLoc = 0 % forward rail button location from nose cone [m]

    launchRailExitVelocityMethod DRSS.core.dynamics.LaunchExitVelocityMethods = DRSS.core.dynamics.LaunchExitVelocityMethods.RAIL_BUTTON_CROSSES_RAIL_TIP

    selfDisablingTimeThreshold = 0.25 % improves accuracy; change if you know what you're doing
  end

  properties (Transient, SetAccess=protected)
    t_launchRailCleared = inf % [s]

    v_launchRailExit = 0 % [m/s]

    launchRailVec % internal use
    launchRailClearanceVec % internal use

    cg_loc_launchRailExit = [inf, inf]

    gravityDynamics % internal use
  end

  properties (Transient, Access=private)
    isCleared = false

    launchRailNormalForce = [0, 0]
  end

  methods
    function this = LaunchRail
      this.disableBoundDynamicsOnTrigger = false;
    end

    function this = setLaunchRailAngleDeg(this, val)
      this.launchRailAngleDeg = val;
    end

    function this = setLaunchRailLength(this, val)
      this.launchRailLength = val;
    end

    function this = setLaunchRailButtonLoc(this, val)
      this.launchRailButtonLoc = val;
    end

    function this = setLaunchRailExitVelocityMethod(this, val)
      this.launchRailExitVelocityMethod = val;
    end

    function this = bindToGravityDynamics(this, gravityDynamics)
      % BINDTOGRAVITYDYNAMICS Adjusts the grounding height of a Gravity Dynamics
      %   object at the beginning of a simulation.
      %
      % See also: DRSS.core.dynamics.Gravity

      arguments
        this DRSS.core.dynamics.LaunchRail
        gravityDynamics DRSS.core.dynamics.Gravity
      end

      this.gravityDynamics = gravityDynamics;
    end
  end

  methods
    function [this, sys, ss0] = resetTransientData(this, sys, ss0)
      [this, sys, ss0] = resetTransientData@DRSS.core.dynamics.IEventTriggerDynamics(this, sys, ss0);

      this.t_launchRailCleared = inf;
      this.v_launchRailExit = 0;

      launchRailUnitVec = [
        sind(this.launchRailAngleDeg) % x
        cosd(this.launchRailAngleDeg) % y
      ];

      this.launchRailVec = launchRailUnitVec .* this.launchRailLength;

      if this.launchRailExitVelocityMethod == DRSS.core.dynamics.LaunchExitVelocityMethods.RAIL_BUTTON_CROSSES_RAIL_TIP
        % When rail button crosses end of rail:
        this.launchRailClearanceVec = launchRailUnitVec .* ...
          (this.launchRailLength - sys.cgX + this.launchRailButtonLoc);
      elseif this.launchRailExitVelocityMethod == DRSS.core.dynamics.LaunchExitVelocityMethods.ROCKET_TIP_CROSSES_RAIL_TIP
        % When tip cross end of rail:
        this.launchRailClearanceVec = launchRailUnitVec .* ...
          (this.launchRailLength - sys.cgX);
      else
        error("Unknown LRE calculation method: %s", this.launchRailExitVelocityMethod);
      end


      [x0, y0] = this.calcCG0(sys);

      if ~isempty(this.gravityDynamics)
        this.gravityDynamics.setGroundingHeight(y0);
      end

      ss0.x = x0;
      ss0.y = y0;
      ss0.theta = deg2rad(this.launchRailAngleDeg);
      ss0.thetad = 0;
      ss0.thetadd = 0;
      ss0.forceConstantTheta = false;
    end

    function [this, sys, massChanged]=step(this, sys, ss)
      massChanged = false;

      y_clear = this.launchRailClearanceVec(2);
      this.isCleared = ss.y > y_clear || ss.t > this.t_launchRailCleared;

      ss.forceConstantTheta = ~this.isCleared;

      if this.isCleared
        % Rocket CLEARED launch rail
        if ss.t < this.t_launchRailCleared
          this.t_launchRailCleared = ss.t;
          this.v_launchRailExit = sqrt(ss.yd^2 + ss.xd^2);
          this.cg_loc_launchRailExit = [ss.x; ss.y];

          this.onEventTrigger(sys, ss);
        elseif ss.t > this.t_launchRailCleared + this.selfDisablingTimeThreshold
          % Disable self
          this.setEnabled(false, sys, ss);
        end
      end
    end
  end

  methods (Access=private)
    function [x0, y0] = calcCG0(this, sys)
      x0 = (sys.len - sys.cgX) * sind(this.launchRailAngleDeg);
      y0 = (sys.len - sys.cgX) * cosd(this.launchRailAngleDeg);
    end
  end

  methods ( Access=protected)
    function evaluateEvent(~, ~)
      % stub
    end
  end
end