classdef ConstantThetaDynamics < DRSS.core.dynamics.Dynamics
  % ConstantThetaDynamics models dynamics that stabilize the theta angle
    % to a constant target angle using a spring-damper system.

  properties
    targetAngleDeg      % Target angle in degrees
    springConstant      % Spring constant (k) controlling the stiffness
    dampingConstant     % Damping constant (c) controlling the damping
  end

  methods
    function this = setTargetAngleDeg(this, val)
      this.targetAngleDeg = val;
    end

    function this = setSpringConstant(this, val)
      this.springConstant = val;
    end

    function this = setDampingConstant(this, val)
      this.dampingConstant = val;
    end
  end

  methods
    function [this, sys, terminate, xdd, ydd, tdd, mdot]=resolve(this, sys, ss)
      terminate = false;

      target = deg2rad(this.targetAngleDeg);

      xdd = 0;
      ydd = 0;
      mdot = 0;

      tdd = this.springConstant * (target - ss.theta) - this.dampingConstant * ss.thetad;
    end
  end
end