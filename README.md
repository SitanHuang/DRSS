![DRSS Logo](doc/assets/logo.png)

# Declarative Rocket Simulation Software

**DRSS** is a MATLAB Domain-Specific Language (DSL) for simulating low-altitude, subsonic rocketry.

Comparing to [OpenRocket](openrocket.info), DRSS offers much greater extensibility and ergonomics for implementing custom event handling and special components, and takes full advantages of MATLAB's mature tooling and environment. Unlike the former, users are not limited to the behaviors and capabilities of a GUI. Being programmable in nature, DRSS enables users to perform [optimization](https://www.mathworks.com/products/optimization.html) of not just rocket design but also flight plan parameters against any arbitrary cost function, which is not possible in OpenRocket. Stage separation, parachutes, and [parallel simulation of any jettisioned stages](#jettisoned-parachute-lander-simulation) are also more powerful and ergonomic. Results generally align between DRSS and other GUI-based softwares, though [aerodynamic calculations](http://cambridgerocket.sourceforge.net/AerodynamicCoefficients.pdf) and wind modeling may have nuanced differences.

## Features

- **Declarative**: An intuitive and expressive syntax to define rocket components, dynamics and scripted events.
- **Accurate Physics**: Research-backed and field-tested models for rocket aerodynamics, motor thrust, and parachute dynamics.
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

Note: The `+VADL` folder contains modules specific to the Vanderbilt Aerospace Design Laboratory and initially developed for VADL's participation in NASA's USLI competitions and can serve as examples or be extended for your own projects.

## Architecture
DRSS is object-oriented and provides plug-and-play abstraction over the physical components, events and dynamics of a rocket system.

### Guidance for Advanced Usage
It is ergonomical to implement custom behaviors in DRSS. Consult in-line documentation below:

- [Solver mechanics](./+DRSS/+solver/@MatlabODESolver/MatlabODESolver.m)
- [Implementing custom dynamics](./+DRSS/+core/+dynamics/@Dynamics/Dynamics.m)
- [Implementing custom events](./+DRSS/+core/+dynamics/@IEventTriggerDynamics/IEventTriggerDynamics.m)

Additionally, the [built-in dynamics objects](./+DRSS/+core/+dynamics/) serve as good examples.

### Core Components

- A **[`System`](./+DRSS/+core/+sim/@System/System.m)** is a special [`MassGroup`](./+DRSS/+core/+obj/@MassGroup/MassGroup.m) that represents a 3 Degree-of-Freedom point mass that is subject to certain dynamics and holds [states data](./+DRSS/+core/+sim/@SystemState/SystemState.m) in the time domain. It serves as the root container for all other components, including masses, dynamics, and events. It can be the actual rocket, a rocket stage, or a jettisioned payload.

- The **[`Mass`](./+DRSS/+core/+obj/@Mass/Mass.m)** class represent a one-dimensional mass span with properties such as moment of inertia, mass, length, center of gravity, and relative location measured from the tip of the parent. Masses should be sequentially appended to a [`MassGroup`](./+DRSS/+core/+obj/@MassGroup/MassGroup.m).

### Events
Events are special `Dynamics` objects that trigger based on certain conditions, such as reaching apogee or a specific altitude. They can enable or disable other `Dynamics` objects and trigger other events. Custom events must implement the [IEventTriggerDynamics](./+DRSS/+core/+dynamics/@IEventTriggerDynamics/IEventTriggerDynamics.m) interface. **Examples:**
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

### Dynamics
A **[`Dynamics`](./+DRSS/+core/+dynamics/@Dynamics/Dynamics.m)** object abstracts the various physical forces, torques and behaviors acting upon a `System`. **Examples**:

  - **[Parachute](./+DRSS/+core/+dynamics/@Parachute/Parachute.m)**: Simulates the deployment and drag of parachutes during descent. Usually triggered by an altitude detection event.
```matlab
drogue = DRSS.core.dynamics.Parachute() ...
    .setDiameter(15 * uc.in_to_m) ...
    .setCD(1.5) ...
    .setN(8) ...
    .setEnabledOnInit(false) ... % usually triggered by an event listener
    .setDeploymentTimeDelay(0.5) ...
    ...
```

  - **[LaunchRail](./+DRSS/+core/+dynamics/@LaunchRail/LaunchRail.m)**: Models the launch rail mechanics and applies initial conditions to `System`.
```matlab
launchRailDynamics = DRSS.core.dynamics.LaunchRail() ...
    .setLaunchRailAngleDeg(5) ...
    .setLaunchRailLength(12 * 12 * uc.in_to_m) ...
    .setLaunchRailButtonLoc(61 * uc.in_to_m) ...
    .setLaunchRailExitVelocityMethod( ...
        DRSS.core.dynamics.LaunchExitVelocityMethods.RAIL_BUTTON_CROSSES_RAIL_TIP) ...
    .bindToGravityDynamics(gravityDynamics) ...
    ...
```

  - **[Gravity](./+DRSS/+core/+dynamics/@Gravity/Gravity.m)**: Models the gravitational force acting on the rocket.
```matlab
gravityDynamics = DRSS.core.dynamics.Gravity() ...
    .setTerminateOnGrounding(true) ...
    .setGroundingTimeThreshold(1) ...
    ...
```

  - **[Motor](./+DRSS/+core/+dynamics/@Motor/Motor.m)**: Models a rocket motor and its thrust profile.
```matlab
motorDynamics = DRSS.core.dynamics.Motor( ...
    "L1400.csv", "L1400", @VADL.vadl_motor_database);
```

  - **[RocketAerodynamics](./+DRSS/+core/+dynamics/@RocketAerodynamics/RocketAerodynamics.m)**: Calculates aerodynamic forces during ascent based on the rocket's geometry and atmospheric conditions.
```matlab
rocketDynamics = DRSS.core.dynamics.RocketAerodynamics( ...
    'D', rocket_diameter, ...
    'L_nose', 7.5 * uc.in_to_m, ...
    ... % other parameters
).recalcTransientParameters(sys);
```

  - Other `Dynamics` objects can be found in [`DRSS.core.dynamics.*`](./+DRSS/+core/+dynamics/)

## Full Usage Examples

### Rocket Ascent to Apogee Simulation

The following script demonstrates how to simulate a rocket's ascent from launch rail to apogee, including the definition of rocket sections, dynamics, and events.

[usage_example.m](./usage_example.m)

### Jettisoned Parachute Lander Simulation

The following script simulates the descent of a payload after jettisoning from the main rocket body.

[payload_example.m](+VADL/+sims/payload_example.m)

# Contributing

All contributions are welcome. Simply initiate pull requests here on Github.

*Note: This project is intended for educational and simulation purposes only. Always follow safety regulations and guidelines when conducting rocketry activities.*