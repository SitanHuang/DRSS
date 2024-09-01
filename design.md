- how to model inertia & something that rotates? has to be rigid section that is not modeled by a general "System"
  - subclass ConstantThetaSystem
    -> rotational dynamics have no effect, theta is always zero

- System extends MassGroup
  - A rigid body point-mass-like 2D area
  - Has: child mass/massgroups, inertia, total projected area
  - Retains: State
    - x, y, theta, params
  - Subject to: translational forces, rotational forces

- rocket geometry needs to reside within one object due to aerodynamics.m