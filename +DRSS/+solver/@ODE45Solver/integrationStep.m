function dsdt = integrationStep(this, t, states, system, resultantParameters)

  dynLen = length(system.dynamicsList);

  systemState = system.systemState.fromStateVec(t, states);


  if this.debugFlag
    fprintf('# integrationStep: t=%.9f, xdd=%.2f, ydd=%.2f, tdd=%.2f, forceConstantTheta=%i\n', t, systemState.xdd, systemState.ydd, systemState.thetadd, systemState.forceConstantTheta);
  end

  if systemState.m == 0 || systemState.I == 0
    recalcInertialProperties(system, systemState);
  end

  % Prep Dynamics

  requestMassRecalc = false;

  for i=1:dynLen
    dyn = system.dynamicsList{i};

    if ~dyn.enabled
      continue;
    end

    [~, ~, massChanged] = dyn.step(system, systemState);

    requestMassRecalc = requestMassRecalc | massChanged;
  end

  % Recalculate mass groups

  if requestMassRecalc || t < systemState.prevTime
    recalcInertialProperties(system, systemState);
  end

  % Execute Dynamics

  terminate = false;
  xdd = 0;
  ydd = 0;
  tdd = 0;
  mdot = 0;

  for i=1:dynLen
    dyn = system.dynamicsList{i};

    if ~dyn.enabled
      if this.debugFlag
        fprintf('RSLV: %30s, DISABLED\n', class(dyn));
      end

      continue;
    end

    [~, ~, tterminate, xxdd, yydd, ttdd, mmdot] = ...
      resolve(dyn, system, systemState);

    terminate = tterminate || terminate;
    xdd = xdd + xxdd;
    ydd = ydd + yydd;
    tdd = tdd + ttdd;
    mdot = mdot + mmdot;

    if this.debugFlag
      fprintf('RSLV: %30s, xdd=%.2f, ydd=%.2f, tdd=%.2f, mdot=%.2f\n', class(dyn), xxdd, yydd, ttdd, mmdot);
    end
  end

  systemState.xdd = xdd;
  systemState.ydd = ydd;
  systemState.thetadd = tdd;
  systemState.mdot = mdot;

  % Post adjustments

  for i=1:dynLen
    dyn = system.dynamicsList{i};

    if ~dyn.enabled
      if this.debugFlag
        fprintf('POST: %30s, DISABLED\n', class(dyn));
      end

      continue;
    end

    [~, ~, tterminate, xxdd, yydd, ttdd, mmdot] = ...
      postAdjustment(dyn, system, systemState);

    terminate = tterminate || terminate;
    systemState.xdd = systemState.xdd + xxdd;
    systemState.ydd = systemState.ydd + yydd;
    systemState.thetadd = systemState.thetadd + ttdd;
    systemState.mdot = systemState.mdot + mmdot;

    if this.debugFlag
      fprintf('POST: %30s, xdd=%.2f, ydd=%.2f, tdd=%.2f, mdot=%.2f\n', class(dyn), xxdd, yydd, ttdd, mmdot);
    end
  end

  if this.debugFlag
    fprintf(systemState.toOneLinerString());
  end

  if systemState.forceConstantTheta
    systemState.thetad = 0;
    systemState.thetadd = 0;
  end

  if this.captureResultantParameters
    resultantParameters.t(end + 1) = t;
    resultantParameters.m(end + 1) = systemState.m;
    resultantParameters.mdot(end + 1) = systemState.mdot;
    resultantParameters.I(end + 1) = systemState.I;
    resultantParameters.equivForceX(end + 1) = systemState.xdd * systemState.m;
    resultantParameters.equivForceY(end + 1) = systemState.ydd * systemState.m;
  end

  dsdt = systemState.toStateVecDeriv();

  systemState.prevTime = systemState.t;

  systemState.terminate = terminate;
end

function recalcInertialProperties(sys, ss)
  sys.recalcMassGroupProperties();
  ss.I = sys.momentOfInertia();
  ss.m = sys.m;
end