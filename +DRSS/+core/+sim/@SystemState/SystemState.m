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

    % non-motion-based states (won't be integrated in solver)
    m = []

    mdot = [] % positive = losing mass

    params = [] % extra system params

    t = []
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
  end
end