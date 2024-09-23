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

    % time variant parameters (won't be integrated in solver and are figured
    % out between dynamics.step and dynamics.resolve)
    m = []
    I = []

    mdot = []

    % derived, time variant parameters
    equivForceX = [];
    equivForceY = [];

    params % extra system params

    t = []

    prevTime = [] % for internal tracking
    forceConstantTheta = [] % internal use, tells solver to force thetad=thetadd=0

    terminate = false % set during runtime to true terminates integration
  end

  methods
    function this = SystemState()
      this.params = containers.Map();
    end

    function copy = makeShallowCopy(this)
      copy = DRSS.core.sim.SystemState();
      copy.x = this.x;
      copy.xd = this.xd;
      copy.xdd = this.xdd;
      copy.y = this.y;
      copy.yd = this.yd;
      copy.ydd = this.ydd;
      copy.theta = this.theta;
      copy.thetad = this.thetad;
      copy.thetadd = this.thetadd;
      copy.m = this.m;
      copy.I = this.I;
      copy.mdot = this.mdot;
      copy.equivForceX = this.equivForceX;
      copy.equivForceY = this.equivForceY;
      copy.t = this.t;
      copy.prevTime = this.prevTime;
      copy.forceConstantTheta = this.forceConstantTheta;
      copy.terminate = this.terminate;
    end

    function total_mem = estimateMemorySize(obj)
      props = properties(obj);

      total_mem = 0;
      for ii=1:length(props)
        % Make shallow copy
        curr_prop = obj.(props{ii});  %#ok<*NASGU>

        s = whos('curr_prop');
        total_mem = total_mem + s.bytes;
      end
    end

    function str = toOneLinerString(this)
      str = sprintf('t: %.2f, x: %.2f, xd: %.2f, xdd: %.2f, y: %.2f, yd: %.2f, ydd: %.2f, theta: %.2f rad, thetad: %.2f, thetadd: %.2f, m: %.2f, I: %.2f, mdot: %.2f, terminate: %s\n', ...
        this.t(end), ...
        this.x(end), this.xd(end), this.xdd(end), ...
        this.y(end), this.yd(end), this.ydd(end), ...
        this.theta(end), this.thetad(end), this.thetadd(end), ...
        this.m(end), this.I(end), this.mdot(end), ...
        mat2str(this.terminate));
    end

    function s = toStateVec(this)
      s = [
        this.x;
        this.y;
        this.xd;
        this.yd;
        this.theta;
        this.thetad
      ];
    end

    function ds = toStateVecDeriv(this)
      ds = [
        this.xd;
        this.yd;
        this.xdd;
        this.ydd;
        this.thetad;
        this.thetadd
      ];
    end

    function this = fromStateVec(this, t, vec)
      this.x = vec(1);
      this.y = vec(2);
      this.xd = vec(3);
      this.yd = vec(4);
      this.theta = vec(5);
      this.thetad = vec(6);
      this.t = t;
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

    function ss = fromSolvedStateVec(t, vec)
      ss = DRSS.core.sim.SystemState();
      ss.x = vec(:, 1);
      ss.y = vec(:, 2);
      ss.xd = vec(:, 3);
      ss.yd = vec(:, 4);
      ss.theta = vec(:, 5);
      ss.thetad = vec(:, 6);
      ss.t = t;
    end
  end
end