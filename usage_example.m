%% Misc Setup

clear;

addpath(fullfile(fileparts(mfilename('fullpath')), 'VADL'))
addpath(fullfile(fileparts(mfilename('fullpath')), 'VADL/Motor Data'))

uc = DRSS.util.unitConv;

%% Define rocket sections & massing

rocket_diameter = 6.125 * uc.in_to_m;

rocketCylindricalInertialGeometry = struct( ...
  'Ri', rocket_diameter / 2 - 0.25 * uc.in_to_m, ...
  'Ro', rocket_diameter / 2 ...
);

noseCone = DRSS.core.obj.Mass("Nose Cone") ...
  .setM(5.1 * uc.lbm_to_kg) ...
  .setLen(8.75 * uc.in_to_m) ...
  .setCGX(10.3300 * uc.in_to_m);
payloadBay = DRSS.core.obj.Mass("Payload bay") ...
  .setM(5.7685 * uc.lbm_to_kg) ...
  .setLen(36 * uc.in_to_m) ...
  .setCGX(22.6789 * uc.in_to_m);
recoveryBay = DRSS.core.obj.Mass("Recovery bay") ...
  .setM(8.5713 * uc.lbm_to_kg) ...
  .setLen(28.5 * uc.in_to_m) ...
  .setCGX(13.586 * uc.in_to_m);
tail = DRSS.core.obj.Mass("Tail") ...
  .setM(2.431 * uc.lbm_to_kg) ...
  .setLen(28 * uc.in_to_m) ...
  .setCGX(9.0820 * uc.in_to_m);

sys = DRSS.core.sim.System("Ascent to Apogee") ...
  .setLaunchSiteElevation(800 * uc.ft_to_m) ... set launch site elevation above mean sea level
  .setLaunchSiteWindSpeed(11 * uc.mph_to_mps) ... reference (base) wind velocity
  .appendChild(noseCone) ...
  .appendChild(payloadBay) ...
  .appendChild(recoveryBay) ...
  .appendChild(tail) ...
  .setInertialGeometryRecursive(rocketCylindricalInertialGeometry);

%% Define Dynamics

gravityDynamics = DRSS.core.dynamics.Gravity() ...
  .setTerminateOnGrounding(true) ...
  .setGroundingTimeThreshold(1); % do not terminate mission even if rocket touches ground during the first second

motorDynamics = DRSS.core.dynamics.Motor( ...
  fullfile(fileparts(mfilename('fullpath')), 'VADL/Motor Data/L1400.csv'), ...
  "L1400", @drss_motor_database);
motor = motorDynamics.genMotorMass(); % the motor Mass is automatically bound to this motorDynamics

sys.appendChild(motor);

L_fin = 80.0 * uc.in_to_m; % length to foremost tip of fins fins from tip of nose cone
L_fin_end = 9.0 * uc.in_to_m; % length from tip of fins to end of section
fin_root_len = 6.5 * uc.in_to_m; % fin root length
fin_tip_len = 2.7 * uc.in_to_m; % fin tip length
fin_semispan_len = 5.8 * uc.in_to_m; % fin semispan
A_fin = fin_semispan_len * (fin_root_len + fin_tip_len) / 2; % fin projected area
A_fin_e = A_fin + (fin_root_len * rocket_diameter) / 2; % fin exposed area

rocketDynamics = DRSS.core.dynamics.RocketAerodynamics( ...
  'D', rocket_diameter, ... rocket diameter
  'L_nose', 7.5 * uc.in_to_m, ... nose cone length
  'L_tail', L_fin + L_fin_end, ... total tail length
  'L_tail_c', 5 * uc.in_to_m, ... boat tail curved section length
  'L_tail_f', 1 * uc.in_to_m, ... boat tail flat section length
  'L_tail_rr', 1 * uc.in_to_m, ... retaining ring length from end of boat tail
  'D_tail_rr', 3.362 * uc.in_to_m, ... retaining ring diameter
  'D_tail', 4 * uc.in_to_m, ... boat tail aft diameter
  'A_fin', A_fin, ... fin projected area
  'A_fin_e', A_fin_e, ... fin exposed area
  's', fin_semispan_len, ... fin semispan
  'cr', fin_root_len, ... fin root length
  'cm', 4 * uc.in_to_m, ... fin midchord length
  'ct', fin_tip_len, ... fin tip length
  't_fin', 0.128 * uc.in_to_m, ... fin thickness
  'L_fin', L_fin, ... length to foremost tip of fins fins from tip of nose cone
  'L_fin_end', L_fin_end, ... length from tip of fins to end of section
  'N_fins', 4, ... number of fins
  'CDr', 0.5, ... hardcoded rocket CD
  'CP', 67.161 * uc.in_to_m ... hardcoded CP
).recalcTransientParameters(sys);

% LaunchRail will automatically set initial state for us
launchRailDynamics = DRSS.core.dynamics.LaunchRail() ...
  .setLaunchRailAngleDeg(5) ...
  .setLaunchRailLength(12 * 12 * uc.in_to_m) ...
  .setLaunchRailButtonLoc(61 * uc.in_to_m) ...
  .setLaunchRailExitVelocityMethod(DRSS.core.dynamics.LaunchExitVelocityMethods.RAIL_BUTTON_CROSSES_RAIL_TIP) ...
  .bindToGravityDynamics(gravityDynamics);

drogue = DRSS.core.dynamics.Parachute() ...
  .setDiameter(15 * uc.in_to_m) ...
  .setCD(1.5) ...
  .setN(8) ... (toroidal = 9, elliptical = 8)
  .setEnabledOnInit(false);
% .deploymentTimeDelay(0);

main = DRSS.core.dynamics.Parachute() ...
  .setDiameter(4 * 12 * uc.in_to_m) ...
  .setCD(2.2) ...
  .setN(9) ... (toroidal = 9, elliptical = 8)
  .setEnabledOnInit(false);

% Legacy: tf_m = 0.7398

% reference area for vertical drag calculations
Ar = (rocketDynamics.A + rocketDynamics.A_side + 0.0077 * 2) / 2; % take average of frontal and side projected areas
CDr = (1.42 * 1.41 * A_fin + 0.56 * rocketDynamics.A_side + 0.0077 * 2) / rocketDynamics.A;

% Legacy: A = 0.019, Ar_side = 0.343, Ar=0.181, CDr = 11.9188

rocketDescentDrag = DRSS.core.dynamics.SimpleDrag() ...
  .setArea(Ar) ...
  .setArea_side(rocketDynamics.A_side) ...
  .setCD(CDr) ...
  .setCD_side(1) ...
  .setEnabledOnInit(false);

%% Define Events

% Apogee event triggers on yd < 0
apogeeListener = DRSS.core.dynamics.events.Apogee() ...
  .setEnabledOnInit(false); % Don't let launch rail jitter trigger false alarm

disableAscentDynamics = DRSS.core.dynamics.events.TriggerOnEnable() ...
  .setDisableBoundDynamicsOnTrigger(true) ...
  .bindTo(rocketDynamics) ...
  .bindTo(apogeeListener);

apogeeListener ...
  .bindTo(disableAscentDynamics) ...
  .bindTo(rocketDescentDrag) ...
  .bindTo(drogue);

% Only start apogee detection when launch rail clears (LaunchRail implements the
% IEventTriggerDynamics abstract class); this is to avoid false triggers due to
% jitters between motor thrust and gravity
launchRailDynamics.bindTo(apogeeListener);

mainDeploymentListener = DRSS.core.dynamics.events.Altitude() ...
  .setAlitude(400 * uc.ft_to_m) ...
  .setEnabledOnInit(false) ...
  .bindTo(main);

apogeeListener.bindTo(mainDeploymentListener);

%% Simulate Rocket Ascent

% System must be the top level MassGroup to calculate
% correctly the sectional moments of inertia; this is
% because non-System MassGroups calculate moments of
% inertia by assuming homogenous distribution in length
sys = sys ...
  .subjectTo(gravityDynamics) ...
  .subjectTo(launchRailDynamics) ...
  .subjectTo(motorDynamics) ...
  .subjectTo(rocketDynamics) ...
  .subjectTo(drogue) ...
  .subjectTo(main) ...
  .subjectTo(rocketDescentDrag) ...
  ... Events:
  .subjectTo(apogeeListener) ...
  .subjectTo(disableAscentDynamics) ...
  .subjectTo(mainDeploymentListener);

% Hyperparameter optimization for ODE45Solver is extremely important; see the
% ODE45Solver.m source code for more detailed information on how to optimize
% both accuracy and computation time by carefully modyfing solver.ODEOptions
solver = DRSS.solver.ODE45Solver(sys) ...
  .setCaptureResultantParameters(false) ... % Whether to capture time variant params (i.e., m, mdot, I), which slows down the solver drastically
  .setPrintPerformanceSummary(true) ...
  .configureODE('RelTol', 1e-3) ... % 1e-3 for prototyping, 1e-8 for reports
  .overrideODEFunc(@ode113); % ode113 (2x more space efficient/less frames, run at 1e-9 rel tol), ode45 (default), ode89 (high precision)

% solver.debugFlag = true;
% solver.setTimeSpan([0 0.5]);

[resultantStates, resultantParameters] = solver.solve();

%% WIP; Debug & dev only:

% sys.momentOfInertia()
% L1400 legacy code: I0=7.4177; DRSS: 7.1587

% plot(resultantStates.t, gradient(resultantStates.yd, resultantStates.t), '-o')
% plot(resultantStates.t, resultantStates.y .* uc.m_to_ft, '-o')
plot(resultantStates.x .* uc.m_to_ft, resultantStates.y .* uc.m_to_ft)
% plot(resultantStates.t, rad2deg(resultantStates.theta), '-o')
% plot(resultantParameters.t, resultantParameters.m .* uc.kg_to_lbm)
% plot(resultantParameters.t, resultantParameters.I)
% plot(resultantParameters.t, resultantParameters.equivForceY, '-o')
% xlim([0 0.1])

% plot([0 launchRailDynamics.launchRailClearanceVec(1)],[0 launchRailDynamics.launchRailClearanceVec(2)], 'r-', 'LineWidth', 3)
hold on
% plot([0 launchRailDynamics.launchRailVec(1)],[0 launchRailDynamics.launchRailVec(2)], 'k-', 'LineWidth', 4)
% % plot(resultantStates.x, resultantStates.y, 'o-')
% xline(launchRailDynamics.systemStateAtTrigger.x .* uc.m_to_ft);
% xline(apogeeListener.systemStateAtTrigger.x .* uc.m_to_ft);
% xline(mainDeploymentListener.systemStateAtTrigger.x .* uc.m_to_ft);
% xline(main.t_complete);
% scatter(launchRailDynamics.cg_loc_launchRailExit(1), launchRailDynamics.cg_loc_launchRailExit(2), 20)

% disp(launchRailDynamics.launchRailClearanceVec)

% DRSS: clears rail at CG=[0.3393, 3.8782]; Legacy: CG=[0.3051, 3.4875]

fprintf("LRE: %.3f fps\n", launchRailDynamics.v_launchRailExit .* uc.mps_to_fps);

fprintf("Descent Time: %.3f s\n", resultantStates.t(end) - apogeeListener.systemStateAtTrigger.t);