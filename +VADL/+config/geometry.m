function sys = geometry(sys)

uc = DRSS.util.unitConv;

sys.configParams.rocket_diameter = 6.170 * uc.in_to_m;

l_fin = ... length to foremost tip of fins from tip of nose cone
  86.509 * uc.in_to_m;

fin_root_len = ... fin root length
  6.75 * uc.in_to_m;

l_fin_end = ... length from tip of fins to end of section
  3.0 * uc.in_to_m + fin_root_len;

fin_tip_len = ... fin tip length
  (6.75 - 4.5) * uc.in_to_m;

fin_semispan_len = ... fin semispan
  6.75 * uc.in_to_m;

A_fin = ... fin projected area
  fin_semispan_len * (fin_root_len + fin_tip_len) / 2;

A_fin_e = ... fin exposed area
  A_fin + (fin_root_len * sys.configParams.rocket_diameter) / 2;

sys.configParams.rocketDynamics = DRSS.core.dynamics.RocketAerodynamics( ...
  'D', sys.configParams.rocket_diameter, ... rocket diameter
  'L_nose', 9.375 * uc.in_to_m, ... nose cone length
  'l_tail', l_fin + l_fin_end, ... length to boat tail from nose tip
  'L_tail_c', 6 * uc.in_to_m, ... boat tail curved section length
  'L_tail_f', 1 * uc.in_to_m, ... boat tail flat section length
  'L_tail_rr', 1 * uc.in_to_m, ... retaining ring length from end of boat tail
  'D_tail_rr', 3.1 * uc.in_to_m, ... retaining ring diameter
  'D_tail', 4 * uc.in_to_m, ... boat tail aft diameter
  'A_fin', A_fin, ... fin projected area
  'A_fin_e', A_fin_e, ... fin exposed area
  's', fin_semispan_len, ... fin semispan
  'cr', fin_root_len, ... fin root length
  'cm', (fin_root_len + fin_tip_len) / 2, ... fin midchord length
  'ct', fin_tip_len, ... fin tip length
  't_fin', 0.118 * uc.in_to_m, ... fin thickness
  'l_fin', l_fin, ... length to foremost tip of fins fins from tip of nose cone
  'N_fins', 4, ... number of fins
  'CDr', 0.314, ... hardcoded rocket CD
  ... 'CDr', 0, ... let DRSS calc CD for us
  'CP', 70.695 * uc.in_to_m ... hardcoded CP (69.488 = DRSS's own calc)
);