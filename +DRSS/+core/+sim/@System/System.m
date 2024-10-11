classdef System < DRSS.core.obj.MassGroup
  %SYSTEM A System is a special type of MassGroup that can be subject to
  %   certain dynamics and holds states data in the time domain
  %
  %   A System obj is a **handle** -> copying a System (either directly or
  %   when passing as value in function calls) creates another reference to
  %   the same object.
  %
  %   A System is much more accurate in calculating Moment of Inertia than
  %   MassGroup as it calculates using the individual sections rather than
  %   using the System's cgX and mass

  properties
    dynamicsList = []

    systemState;

    IAarray = {}

    % Launch site elevation above mean sea level [m]
    launchSiteElevation = 240
    % launch site air temperature [C]
    launchSiteTemp = 26.67;
    % launch site pressure [Pa]
    launchSitePressure = 101592;
    % launch site reference wind speed [m/s]
    launchSiteWindSpeed = 0;

    % associated parameters freely available to users
    configParams = struct();
  end

  methods
    function this = System(varargin)
      this = this@DRSS.core.obj.MassGroup(varargin);

      this.systemState = DRSS.core.sim.SystemState.createZeroState();
    end

    function this=setLaunchSiteElevation(this, val)
      this.launchSiteElevation = val;
    end
    function this=setLaunchSiteTemp(this, val)
      this.launchSiteTemp = val;
    end
    function this=setLaunchSitePressure(this, val)
      this.launchSitePressure = val;
    end
    function this=setLaunchSiteWindSpeed(this, val)
      this.launchSiteWindSpeed = val;
    end

    function this=setSystemState(this, val)
      this.systemState = val;
    end
  end

  methods
    function this = subjectTo(this, dynamics)
      this.dynamicsList = [this.dynamicsList {dynamics}];
    end

    function momentOfInertia=momentOfInertia(this)
      momentOfInertia = 0;

      lengthCursor = 0;

      this.IAarray = {};

      for i = 1:length(this.massList)
        currentMass = this.massList{i};

        I = currentMass.calcMomentOfInertia( ...
          this.cgX, ... vehicle CG
          lengthCursor + currentMass.cgX + currentMass.offset ... length from nose cone to section cg
        );

        this.IAarray{i} = {currentMass.label, 'I',I,'cg0',this.cgX,'l_cg_i',lengthCursor + currentMass.cgX + currentMass.offset};

        lengthCursor = lengthCursor + currentMass.len + currentMass.offset;

        momentOfInertia = momentOfInertia + I;
      end
    end
  end
end
