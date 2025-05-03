classdef SimpleDragPlate < DRSS.core.dynamics.Dynamics
  % SIMPLEDRAGPLATE  A DRSS Dynamics module that deploys a concentric drag
  %   plate once the rocket passes a user‑defined trigger function.
  %
  %   The plate is assumed to be coaxial with the body and located on the rocket
  %   such that it produces pure drag with negligible trimming moment. A flat,
  %   round plate normal to the freestream is modelled with a user-defined drag
  %   coefficient function that takes in the current system state. The forces
  %   are resolved in the earth‑fixed X–Y plane using the instantaneous relative
  %   wind vector.

  properties
    % A function that takes in the SystemState as input argument and returns
    % true/false for whether plate should be deployed. Deployment is one-way trip.
    %
    %   @(ss: SystemState) -> boolean
    %
    triggerFunc = @(ss) false

    % A function that takes in the SystemState as input argument and returns
    % the airbrake CD.
    %
    %   @(ss: SystemState) -> number
    %
    CDFunc = @(ss) 0.6
    area = 0.05 % m^2, default value is about 0.5 ft^2

    fillTime = inf % drag plate deployment time [s]

    deploymentTimeDelay = 0; % deployment time offset after trigger [s]
  end

  properties (Transient, SetAccess=protected)
    t_deploy = inf % time at deployment start [s]

    t_complete = inf % time at deployment complete [s]

    systemStateAtTrigger;
  end

  methods
    function this = setTriggerFunc(this, val)
      this.triggerFunc = val;
    end
    function this = setCDFunc(this, val)
      this.CDFunc = val;
    end
    function this = setArea(this, val)
      this.area = val;
    end
    function this = setFillTime(this, val)
      this.fillTime = val;
    end
    function this = setDeploymentTimeDelay(this, val)
      this.deploymentTimeDelay = val;
    end
  end

  methods
    function [this, sys, terminate, xdd, ydd, tdd, mdot]=resolve(this, sys, ss)
      terminate = false;

      xdd = 0;
      ydd = 0;
      tdd = 0;
      mdot = 0;

      this.checkDeploymentState(sys, ss);

      tElapsed = ss.t - this.t_deploy;

      if tElapsed <= 0
        return;
      end

      % Transient opening (linear interp):
      if ss.t < this.t_complete
        frac = (ss.t - this.t_deploy) / this.fillTime; % 0 -> 1
      else
        frac = 1.0;
      end

      vRelX = ss.xd + ss.windSpeed;
      vRelY = ss.yd;
      V = hypot(vRelX, vRelY);

      if V < 0.5 % about 1mph cutoff
        return;
      end

      rho = ss.airDensity;

      % aerodynamic forces

      % we're gonna ignore any pitching/asymmetric drag effects, prob needs CFD
      % anyways for non-straight-on orientations

      CD = this.CDFunc(ss);

      Fd = 0.5 * rho * V^2 * CD * this.area * frac; % drag magnitude

      uX = -vRelX / V;
      uY = -vRelY / V;
      Fx = Fd * uX;
      Fy = Fd * uY;

      xdd = Fx / ss.m;
      ydd = Fy / ss.m;
    end
  end

  methods (Access=protected)
    function [this, sys, ss] = checkDeploymentState(this, sys, ss)
      if ss.t >= this.t_complete
        return;
      end

      if ~this.triggerFunc(ss)
        this.t_deploy = inf;
        this.t_complete = inf;
        return;
      end

      this.t_deploy = min(this.t_deploy, this.t_lastEnable + this.deploymentTimeDelay);
      this.t_complete = this.t_deploy + this.fillTime;

      this.systemStateAtTrigger = ss.makeShallowCopy();
    end
  end
end