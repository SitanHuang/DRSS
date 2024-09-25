classdef ConstantThetadDynamics < DRSS.core.dynamics.Dynamics
  % ConstantThetaDynamics models dynamics that stabilize the angular
  % velocity to a constant target using a spring-damper system.

  properties
    targetThetad      % Target angular velocity in rad/s
    dampingConstant     % Damping constant (c) controlling the damping
  end

  methods
    function this = setTargetThetad(this, val)
      this.targetThetad = val;
    end

    function this = setDampingConstant(this, val)
      this.dampingConstant = val;
    end
  end

  methods
    function [this, sys, terminate, xdd, ydd, tdd, mdot]=resolve(this, sys, ss)
      terminate = false;

      target = deg2rad(this.targetThetad);

      xdd = 0;
      ydd = 0;
      mdot = 0;

      tdd = this.dampingConstant * (target - ss.thetad);
    end
  end
end