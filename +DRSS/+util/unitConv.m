function unitConv=unitConv

unitConv = struct;

unitConv.ft_to_m = 0.3048;
unitConv.ft2_to_m2 = unitConv.ft_to_m^2;
unitConv.in_to_m = 1 / 39.3701;
unitConv.in2_to_m2 = unitConv.in_to_m^2;
unitConv.m_to_in = 1/unitConv.in_to_m;
unitConv.m_to_ft = 3.28084;
unitConv.m2_to_ft2 = unitConv.m_to_ft^2;
unitConv.lbm_to_kg = 0.453592;
unitConv.oz_to_kg = 0.02835;
unitConv.kg_to_lbm = 1/unitConv.lbm_to_kg;
unitConv.rad_to_deg = 57.2958;
unitConv.deg_to_rad = 1/unitConv.rad_to_deg;
unitConv.mph_to_mps = 0.44704;
unitConv.mph_to_fps = 1.467;
unitConv.mps2_to_G = 0.101971621;
unitConv.N_to_lbf = 0.2248089431;
unitConv.J_to_ftlbf = 0.737562;
unitConv.G_to_fps2 = 32.174;
unitConv.atm_to_Pa = 101325;
unitConv.Pa_to_atm = 1/unitConv.atm_to_Pa;

unitConv.mps_to_fps = 3.28084;