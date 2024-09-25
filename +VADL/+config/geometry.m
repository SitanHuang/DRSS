function sys = geometry(sys)

uc = DRSS.util.unitConv;

sys.configParams.rocket_diameter = 6.125 * uc.in_to_m;

l_fin = ... length to foremost tip of fins from tip of nose cone
  80.0 * uc.in_to_m;

l_fin_end = ... length from tip of fins to end of section
  9.0 * uc.in_to_m;

fin_root_len = ... fin root length
  6.5 * uc.in_to_m;

fin_tip_len = ... fin tip length
  2.7 * uc.in_to_m;

fin_semispan_len = ... fin semispan
  5.8 * uc.in_to_m;

A_fin = ... fin projected area
  fin_semispan_len * (fin_root_len + fin_tip_len) / 2;

A_fin_e = ... fin exposed area
  A_fin + (fin_root_len * sys.configParams.rocket_diameter) / 2;

sys.configParams.rocketDynamics = DRSS.core.dynamics.RocketAerodynamics( ...
  'D', sys.configParams.rocket_diameter, ... rocket diameter
  'L_nose', 7.5 * uc.in_to_m, ... nose cone length
  'l_tail', l_fin + l_fin_end, ... length to boat tail from nose tip
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
  'l_fin', l_fin, ... length to foremost tip of fins fins from tip of nose cone
  'N_fins', 4, ... number of fins
  'CDr', 0.5, ... hardcoded rocket CD
  'CP', 47.161 * uc.in_to_m ... hardcoded CP
);