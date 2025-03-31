classdef Gravity < DRSS.core.dynamics.Dynamics
  % GRAVITY Models gravitational effects acting on a system based on altitude
  %   and includes handling grounding events.
  %
  %   See also: DRSS.core.dynamics.Dynamics

  properties (Constant)
    % RE Earth radius in meters.
    RE = 6.37e6

    % G0 Gravitational acceleration at mean sea level in m/s^2.
    g0 = 9.80665
  end

  % Static properties emulated as static getter methods
  methods (Static)
    % Key in the SystemState.params Map obj to access the current gravitational acceleration in m/s^2
    function key = SYSTEMSTATE_PARAM_KEY_CURRENT_GRAV_ACC
      key = 'CurrentGravAcc';
    end

    % Key in the SystemState.params Map obj to denote grounding
    function key = SYSTEMSTATE_PARAM_KEY_CURRENT_GROUNDING
      key = 'CurrentGrounding';
    end
  end

  properties
    % GROUNDINGHEIGHT Height of ground in meters.
    %
    %   Specifies the height of the ground relative to the base coordinate frame
    %   used in simulation for which ground forces begin to occur.
    %
    %   Default: 0 meters
    groundingHeight = 0

    % GROUNDINGTHRESHOLD Tolerance for finite penetration of the ground in meters.
    %
    %   Allows for a specified tolerance in meters below which the system is
    %   considered grounded but does not yet enact any forces.
    %
    %   Default: 1e-6 meters
    groundingThreshold = 1e-6

    % GROUNDINGTIMETHRESHOLD Time threshold in seconds for grounding consideration.
    %
    %   Specifies a time threshold during which grounding is not considered
    %   a mission-ending event, allowing for transient interactions with the
    %   ground such as hard landings or bounces at the beginning of a simulation.
    %
    %   Default: 0.5 seconds
    groundingTimeThreshold = 0.5

    % TERMINATEONGROUNDING Flag to determine whether to terminate the mission on grounding.
    %
    %   A boolean flag that, when set to true, causes the simulation to terminate
    %   if grounding occurs (taking into account the groundingThreshold and
    %   groundingTimeThreshold).
    %
    %   Default: true
    terminateOnGrounding = true
  end

  methods
    function this=setTerminateOnGrounding(this, val)
      this.terminateOnGrounding = val;
    end

    function this=setGroundingHeight(this, val)
      this.groundingHeight = val;
    end

    function this=setGroundingThreshold(this, val)
      this.groundingThreshold = val;
    end

    function this=setGroundingTimeThreshold(this, val)
      this.groundingTimeThreshold = val;
    end

    function [this, sys, ss0] = resetTransientData(this, sys, ss0)
      [this, sys, ss0] = resetTransientData@DRSS.core.dynamics.Dynamics(this, sys, ss0);

      ss0.declareCustomParam(this.SYSTEMSTATE_PARAM_KEY_CURRENT_GRAV_ACC);
      ss0.declareCustomParam(this.SYSTEMSTATE_PARAM_KEY_CURRENT_GROUNDING);
    end

    function [this, sys, massChanged]=step(this, sys, ss)
      gravAcc = -this.g0 * (this.RE / (this.RE + sys.launchSiteElevation + ss.y))^2;
      ss.params.(this.SYSTEMSTATE_PARAM_KEY_CURRENT_GRAV_ACC) = gravAcc;

      grounding = abs(ss.y - this.groundingHeight) <= this.groundingThreshold;
      ss.params.(this.SYSTEMSTATE_PARAM_KEY_CURRENT_GROUNDING) = grounding;

      massChanged = false;
    end

    function [this, sys, terminate, xdd, ydd, tdd, mdot]=resolve(this, sys, ss)
      terminate = false;

      xdd = 0;
      ydd = 0;
      tdd = 0;
      mdot = 0;

      if abs(ss.y - this.groundingHeight) < this.groundingThreshold % tolerance for some finite penetration of ground
        return;
      end

      if ss.y < this.groundingHeight
        % Rocket below ground, it's OVER

        if ss.t >= this.groundingTimeThreshold
          terminate = this.terminateOnGrounding;
        end

        return;
      end

      g = this.g0 * (this.RE / (this.RE + sys.launchSiteElevation + ss.y))^2;


      if ss.forceConstantTheta
        ydd = -g * (sin(ss.theta)^2 - 1);
        xdd = g * cos(ss.theta) * sin(ss.theta);
      else
        ydd = -g;
      end
    end
  end

  methods (Static)
    function g = calcGravity(sys, ss)
      g0 = DRSS.core.dynamics.Gravity.g0;
      RE = DRSS.core.dynamics.Gravity.RE;

      g = -g0 * (RE / (RE + sys.launchSiteElevation + ss.y))^2;
    end
    function gravAcc = getCurrentGravAccFromSystemState(ss)
      gravAcc = nan;

      if isfield(ss.params, DRSS.core.dynamics.Gravity.SYSTEMSTATE_PARAM_KEY_CURRENT_GRAV_ACC())
        gravAcc = ss.params.(DRSS.core.dynamics.Gravity.SYSTEMSTATE_PARAM_KEY_CURRENT_GRAV_ACC);
      end
    end

    function grounding = getCurrentGroundingFromSystemState(ss)
      grounding = nan;

      if isfield(ss.params, DRSS.core.dynamics.Gravity.SYSTEMSTATE_PARAM_KEY_CURRENT_GROUNDING())
        grounding = ss.params.(DRSS.core.dynamics.Gravity.SYSTEMSTATE_PARAM_KEY_CURRENT_GROUNDING);
      end
    end
  end
end