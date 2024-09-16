function dsdt = integrationStep(this, t, states, system)
  dynLen = length(system.dynamicsList);

  systemState = system.systemState.fromStateVec(t, states);

  if systemState.m == 0 || systemState.I == 0
    recalcMass(system, systemState);
  end

  % Prep Dynamics

  for i=1:dynLen
    dyn = system.dynamicsList{i};
    dyn.resetTransientData();
  end

  for i=1:dynLen
    dyn = system.dynamicsList{i};
    dyn.step(system, systemState);
  end

  % Recalculate mass groups

  recalcMass(system, systemState);

  % Execute Dynamics

  terminate = false;
  xdd = 0;
  ydd = 0;
  tdd = 0;
  mdot = 0;

  for i=1:dynLen
    dyn = system.dynamicsList{i};

    [~, ~, tterminate, xxdd, yydd, ttdd, mmdot] = ...
      resolve(dyn, system, systemState);

    terminate = tterminate || terminate;
    xdd = xdd + xxdd;
    ydd = ydd + yydd;
    tdd = tdd + ttdd;
    mdot = mdot + mmdot;

    if this.debugFlag
      fprintf('%30s, xdd=%.2f, ydd=%.2f, tdd=%.2f, mdot=%.2f\n', class(dyn), xxdd, yydd, ttdd, mmdot);
    end
  end

  systemState.xdd = xdd;
  systemState.ydd = ydd;
  systemState.thetadd = tdd;
  systemState.mdot = mdot;

  if this.debugFlag
    fprintf(systemState.toOneLinerString());
  end

  dsdt = systemState.toStateVecDeriv();

  systemState.terminate = terminate;
end

function recalcMass(sys, ss)
  sys.recalcMassGroupProperties();
  ss.I = sys.momentOfInertia();
  ss.m = sys.m;
end