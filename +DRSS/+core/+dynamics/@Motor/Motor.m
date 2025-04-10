classdef Motor < DRSS.core.dynamics.Dynamics
  properties
    % the MassGroup representing the motor, bound and controlled by Motor dynamics
    motorMassGroup


    % The offset of generated motor mass towards the end of rocket in meters.
    %
    % Default: 1 in = 0.0254 m
    motorOffset = 0.0254 % [m]

    % The amount of thrust retained after accounting for non-idealistic factors
    % (ex. guide rail friction, wobbly rocket body, or poor motor specs)
    lossFactor = 1

    % only output force when thrust is known to completely replace the
    % normal force; reduces jitter but may have performance implications;
    % on the other hand, jitter may increase the number of frames ODE45
    % needs to calculate
    %
    % Default: true
    adjustForGrounding = true

    t0 = 0
  end

  properties
    motor_data
    motortype
    motor_params
  end

  properties (Transient)
    motor_state_temp
  end

  methods
    function this = Motor(motor_csv_path, motortype, motor_database_func)
      this = this.motor_load(motor_csv_path, motortype, motor_database_func);

      this.motortype = motortype;
    end

    function this = setAdjustForGrounding(this, val)
      this.adjustForGrounding = val;
    end

    function this = setLossFactor(this, val)
      this.lossFactor = val;
    end

    function this = setIgnitionTimestamp(this, t0)
      this.t0 = t0;
    end

    function this = setMotorOffset(this, motorOffset)
      this.motorOffset = motorOffset;
    end

    function mg = genMotorMass(this)
      % GENMOTORMASS Generates a MassGroup representing the motor with an offset
      %   equaling -(length of motor); also binds the motor mass to this Motor dynamics
      mg = DRSS.core.obj.MassGroup(join([this.motortype 'Motor']));

      this = this.bindToMass(mg);

      mg ...
        .overrideLen(this.motor_params.L) ...
        .overrideCGX(this.motor_params.L / 2) ...
        .overrideM(this.motor_params.m0 + this.motor_params.m_prop0) ...
        .setOffset(-this.motor_params.L + this.motorOffset) ...
        .setInertialGeometry(struct('R', this.motor_params.D / 2)) ...
        .setMomentOfInertiaMethod(DRSS.core.obj.MomentOfInertiaMethod.SolidCylinderApproximation) ...
        .lockGeometry();
    end

    function this = bindToMass(this, massGroup)
      if ~isa(massGroup,'DRSS.core.obj.MassGroup')
        error("bindToMotor() requires a MassGroup as argument");
      end

      if max([massGroup.m, massGroup.cgX, massGroup.len]) > 0
        warning("bindToMotor() requires an empty massGroup to work with");
      end

      this.motorMassGroup = massGroup;
    end

    function [this, sys, ss0] = resetTransientData(this, sys, ss0)
      [this, sys, ss0] = resetTransientData@DRSS.core.dynamics.Dynamics(this, sys, ss0);

      ss0.declareCustomParam("ThrustToWeight");
      ss0.declareCustomParam("Thrust");
    end

    function [this, sys, massChanged]=step(this, sys, ss)
      % calculates thrust & updates motor MassGroup:
      [thrust, mdot] = this.motor_update_state(ss);

      massChanged = mdot ~= 0;

      this.motor_state_temp = [thrust, mdot];
    end

    function [this, sys, terminate, xdd, ydd, tdd, mdot_actual]=resolve(this, sys, ss)
      terminate = false;

      xdd = 0;
      ydd = 0;
      tdd = 0;

      thrust = this.motor_state_temp(1);
      mdot = -this.motor_state_temp(2);
      mdot_actual = -mdot;

      if thrust == 0 || mdot == 0
        return;
      end

      Th_n = mdot * ss.thetad * (sys.len - sys.cgX);

      % thrust vectors (only real component of Th_t is taken because at very
      % small Th/mdot the argument of square root may become negative)
      Th_t = mdot * sqrt((thrust/mdot)^2 - ((sys.len - sys.cgX) * ss.thetad)^2);
      Th_t = real(Th_t);

      xdd = (Th_n/ss.m) * cos(ss.theta) + (Th_t/ss.m) * sin(ss.theta);
      ydd = ((-Th_n)/ss.m) * sin(ss.theta) + (Th_t/ss.m) * cos(ss.theta);
      tdd = -(mdot * ss.thetad * (sys.len - sys.cgX)^2) / ss.I;

      ss.params.ThrustToWeight = thrust / 9.8 / ss.m; % TODO: should use variable gravity
      ss.params.Thrust = thrust; % TODO: should use variable gravity

      if this.adjustForGrounding
        gravAcc = DRSS.core.dynamics.Gravity.getCurrentGravAccFromSystemState(ss);
        grounding = DRSS.core.dynamics.Gravity.getCurrentGroundingFromSystemState(ss);

        if grounding && ydd + gravAcc <= 0
          % rocket on ground and thrust hasn't exceeded gravitational pull;
          % there shouldn't be any movement as thrust simply substitutes a
          % portion of the normal force
          ydd = 0;
          xdd = 0;
        end
      end
    end
  end

  methods (Access=private)
    function this = motor_load(this, motor_csv_path, motortype, motor_database_func)
      this.motor_data = readcell(motor_csv_path);
      this.motor_data = cell2mat(this.motor_data(2:end,:));

      [L_motor, D_motor, m_motor0, m_prop0, ~] = motor_database_func(motortype);

      this.motor_params = struct( ...
        'L', L_motor, ...
        'D', D_motor, ...
        'm0', m_motor0 - m_prop0, ...
        'm_prop0', m_prop0,...
        't_burn', this.motor_data(end, 1) ...
      );
    end

    function [Th, mdot] = motor_update_state(this, sysState)
      Th = 0;
      mdot = 0;

      Telapsed = sysState.t - this.t0;
      Tend = this.t0 + this.motor_params.t_burn;

      % after burn time
      if sysState.t >= Tend || sysState.t < this.t0
        return;
      end

      % thrust
      Th = interp1(this.motor_data(:,1),this.motor_data(:,2), Telapsed);
      if isnan(Th)
        Th = 0;
      end

      % propellant mass (linearly decreasing)
      mdot = -this.motor_params.m_prop0 / this.motor_params.t_burn;
      m_prop = mdot * Telapsed;

      % total motor mass
      m_motor = this.motor_params.m0 + this.motor_params.m_prop0 + m_prop;

      this.motorMassGroup.overrideM(m_motor);

      Th = Th * this.lossFactor;
    end
  end
end