![DRSS Logo](doc/assets/logo.png)

# Declarative Rocket Simulation Software

**DRSS** is a MATLAB Domain-Specific Language (DSL) for simulating low-altitude rocketry with elegance and precision. Originally made for the [Vanderbilt Aerospace Design Laboratory (VADL)](https://www.vadl.org/), DRSS was designed to improve rapid protyping capabilities during flight plan development.

## Features

- **Declarative**: An intuitive and expressive syntax to define rocket components and dynamics.
- **Accurate Physics**: Research-backed and field-tested models for rocket aerodynamics, motor, and parachute dynamics.
- **Event Handling**: Complex event-driven behaviors like stage separation and parachute deployment.
- **Solver-agnostic**: Choose from any MATLAB's built-in ODE solvers and customize their parameters to suit your needs.
- **Modularity**: Easily extend and customize simulations with object-oriented programming.


## Table of Contents

- [Features](#features)
- [Getting Started](#getting-started)
- [Architecture](#architecture)
- [Full Usage Examples](#full-usage-examples)
- [Contributing](#contributing)
- [License](./LICENSE)

## Getting Started

1. **Clone the Repository**

   ```bash
   git clone https://github.com/SitanHuang/DRSS.git
   ```

2. **Add to MATLAB Path**

   ```matlab
   addpath(genpath('path_to_DRSS'));
   ```

Note: The `+VADL` folder contains modules specific to the Vanderbilt Aerospace Design Laborator and initially developed for VADL's participation in NASA's USLI competitions and can serve as examples or be extended for your own projects.

## Architecture
DRSS uses an object-oriented architecture that mirrors the physical components and dynamics of a rocket system. This allows for an extensible and flexible framework where each component can be individually customized and interacted with.

### Core Components

#### System
The `System` class is a special `MassGruop` object that represents a point mass that can move about 2-D space. It serves as the root container for all other components, including masses, dynamics, and events. **Example:**
```matlab
sys = DRSS.core.sim.System("Ascent to Apogee") ...
    .setLaunchSiteElevation(800 * uc.ft_to_m) ...
    .setLaunchSiteWindSpeed(11 * uc.mph_to_mps);
```

#### Mass and MassGroups

  - **Mass**: The `Mass` class represents a physical mass element of the rocket, such as the nose cone, payload bay, or recovery bay. Each mass can have properties including mass (`m`), length (`len`), center of gravity (`CGX`), and offset (`offset`), measured from the tip of the parent `MassGroup` (e.g., a rocket). Masses should be sequentially appended to a `MassGroup`, starting from the tip and extending towards the rear.
  - **MassGroup**: The `MassGroup` class is a collection of `Mass` objects. It is useful for representing sections of the rocket that move or interact together. **Example:**
```matlab
noseCone = DRSS.core.obj.Mass("Nose Cone") ...
    .setM(5.1 * uc.lbm_to_kg) ...
    .setLen(8.75 * uc.in_to_m) ...
    .setCGX(10.3300 * uc.in_to_m);
```

#### Dynamics
The `Dynamics` classes abstract the various physical forces and behaviors acting upon a `System`. **Examples**:
  - **Gravity**: Models the gravitational force acting on the rocket.
```matlab
gravityDynamics = DRSS.core.dynamics.Gravity() ...
    .setTerminateOnGrounding(true) ...
    .setGroundingTimeThreshold(1);
```

  - **Motor**: Models a rocket motor and its thrust profile.
```matlab
motorDynamics = DRSS.core.dynamics.Motor( ...
    "L1400.csv", "L1400", @VADL.vadl_motor_database);
```

  - **RocketAerodynamics**: Calculates aerodynamic forces during ascent based on the rocket's geometry and atmospheric conditions.
```matlab
rocketDynamics = DRSS.core.dynamics.RocketAerodynamics( ...
    'D', rocket_diameter, ...
    'L_nose', 7.5 * uc.in_to_m, ...
    ... % other parameters
).recalcTransientParameters(sys);
```

  - **Parachute**: Simulates the deployment and drag of parachutes during descent.
```matlab
drogue = DRSS.core.dynamics.Parachute() ...
    .setDiameter(15 * uc.in_to_m) ...
    .setCD(1.5) ...
    .setN(8) ...
    .setEnabledOnInit(false);
```

  - **LaunchRail**: Models the launch rail mechanics and initial conditions.
```matlab
launchRailDynamics = DRSS.core.dynamics.LaunchRail() ...
    .setLaunchRailAngleDeg(5) ...
    .setLaunchRailLength(12 * 12 * uc.in_to_m) ...
    .setLaunchRailButtonLoc(61 * uc.in_to_m) ...
    .setLaunchRailExitVelocityMethod( ...
        DRSS.core.dynamics.LaunchExitVelocityMethods.RAIL_BUTTON_CROSSES_RAIL_TIP) ...
    .bindToGravityDynamics(gravityDynamics);
```

#### Events
Events are special `Dynamics` objects triggers based on certain conditions, such as reaching apogee or a specific altitude. They can enable or disable other `Dynamics` objects and trigger other events. **Example:**
```matlab
apogeeListener = DRSS.core.dynamics.events.Apogee() ...
    .setEnabledOnInit(false);

disableAscentDynamics = DRSS.core.dynamics.events.TriggerOnEnable() ...
    .setDisableBoundDynamicsOnTrigger(true) ...
    .trigger(rocketDynamics) ...
    .trigger(apogeeListener);

apogeeListener ...
    .trigger(disableAscentDynamics) ...
    .trigger(rocketDescentDrag) ...
    .trigger(drogue);
```

# Full Usage Examples

## Ascent to Apogee Simulation

The following file demonstrates how to simulate a rocket's ascent to apogee, including the definition of rocket sections, dynamics, and events like parachute deployment.

[usage_example.m](./usage_example.m)

## Payload Descent Simulation

The following file simulates the descent of a payload after jettisoning from the main rocket body.

[payload.m](+VADL/+sims/payload.m)

# Contributing

I welcome all contributions from the community. Simply initiate pull requests here on Github.

*Note: This project is intended for educational and simulation purposes only. Always follow safety regulations and guidelines when conducting rocketry activities.*