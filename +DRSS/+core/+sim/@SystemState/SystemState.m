classdef SystemState < handle
  properties
    x = []
    xd = []
    xdd = []

    y = []
    yd = []
    ydd = []

    theta = [] % rad
    thetad = []
    thetadd = []

    % non-motion-based states (won't be integrated in solver and are figured
    % out between dynamics.step and dynamics.resolve)
    m = []
    I = []

    mdot = [] % positive = losing mass

    params = [] % extra system params

    t = []
  end

  methods
    function s = toStateVec(this)
      s = [
        this.x;
        this.y;
        this.xd;
        this.yd;
        this.theta;
        this.thetad;
        1 % Continuation flag: if set to 0, simulation terminates
      ];
    end
  end

  methods(Static)
    function ss = createZeroState()
      ss = DRSS.core.sim.SystemState();
      ss.x = 0;
      ss.xd = 0;
      ss.xdd = 0;

      ss.y = 0;
      ss.yd = 0;
      ss.ydd = 0;

      ss.theta = 0;
      ss.thetad = 0;
      ss.thetadd = 0;

      ss.m = 0;

      ss.mdot = 0;

      ss.t = 0;
    end

    function ss = fromStateVector(t, vec)
      ss = SystemState();
      ss.x = vec(1);
      ss.y = vec(2);
      ss.xd = vec(3);
      ss.yd = vec(4);
      ss.theta = vec(5);
      ss.thetad = vec(6);
      ss.m = vec(7);
      ss.t = t;
  end
  end
end