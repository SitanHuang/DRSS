classdef LaunchRail < DRSS.core.dynamics.Dynamics
  % LAUNCHRAIL A Dynamics object that sets the initial state of the system and
  %   forces theta to be constant while a System is on the launch rail.

  properties
    launchRailAngleDeg = 0 % launch angle [deg]
    launchRailLength % launch rail length [m]

    launchRailButtonLoc = 0 % forward rail button location from nose cone [m]

    launchRailExitVelocityMethod DRSS.core.dynamics.LaunchExitVelocityMethods = DRSS.core.dynamics.LaunchExitVelocityMethods.RAIL_BUTTON_CROSSES_RAIL_TIP
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
      % BINDTOGRAVITYDYNAMICS Adjusts the grounding height of a Gravity Dyanmics
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
    end

    function [this, sys, terminate, xdd, ydd, tdd, mdot] = postAdjustment(this, sys, ss)
      terminate = false;
      xdd = 0;
      ydd = 0;
      tdd = 0;
      mdot = 0;

      if this.isCleared
        % Rocket CLEARED launch rail
        if ss.t < this.t_launchRailCleared
          this.t_launchRailCleared = ss.t;
          this.v_launchRailExit = sqrt(ss.yd^2 + ss.xd^2);
          this.cg_loc_launchRailExit = [ss.x; ss.y];
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
end