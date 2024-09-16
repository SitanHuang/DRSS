classdef Gravity < DRSS.core.dynamics.Dynamics
  % GRAVITY Models gravitational effects acting on a system based on altitude
  %   and includes handling grounding events.
  %
  %   See also: DRSS.core.dynamics.Dynamics

  properties (Constant)
    % RE Earth radius in meters.
    RE = 6.37e6

    % G0 Gravitational acceleration at mean sea level in meters per second squared.
    g0 = 9.80665
  end

  properties
    % eps Launch site elevation above mean sea level in meters.
    eps = 0

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
    %   Default: 0 meters
    groundingThreshold = 0

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
    function this=setEPS(this, eps)
      this.eps = eps;
    end

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

      ydd = -this.g0 * (this.RE / (this.RE + this.eps + ss.y))^2;
    end
  end
end