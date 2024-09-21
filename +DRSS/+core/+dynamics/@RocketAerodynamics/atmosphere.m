function [rho,T,p,mu] = atmosphere(z,T0,p0,R,B,g0)
% Atmospheric Conditions

% ----------------------------------------------------------------------------
% Calculates the density, dynamic viscosity, and temperature of air at a
% given altitude.

% INPUT(S)= z - altitude above launch site [m]
%           T0 - air temperature at launch site [K]
%           p0 - air pressure at launch site [Pa]
%           R - ideal gas constant for air [J/(kg.K)]
%           B - temperature lapse rate in the troposphere [K/m]
%           mu_data - array storing dynamic viscosity [Pa.s] as a function
%                     of temperature [K]

% OUTPUT(S)= rho - air density [kg/m^3]
%            T - air temperaure [K]
%            p - air pressure [Pa]
%            mu - air dynamic viscosity [Pa.s]

    % original data:
    mu_data_old = [
        198.15	0.00001318
        223.15	0.00001456
        248.15	0.00001588
        258.15	0.0000164
        263.15	0.00001665
        268.15	0.0000169
        273.15	0.00001715
        278.15	0.0000174
        283.15	0.00001764
        288.15	0.00001789
        293.15	0.00001813
        298.15	0.00001837
        303.15	0.0000186
        313.15	0.00001907
    ];

    % DRSS new data:
    mu_data = [
        198.150000000000	1.31800000000000e-05
        203.150000000000	1.34560000000000e-05
        208.150000000000	1.37320000000000e-05
        213.150000000000	1.40080000000000e-05
        218.150000000000	1.42840000000000e-05
        223.150000000000	1.45600000000000e-05
        228.150000000000	1.48240000000000e-05
        233.150000000000	1.50880000000000e-05
        238.150000000000	1.53520000000000e-05
        243.150000000000	1.56160000000000e-05
        248.150000000000	1.58800000000000e-05
        253.150000000000	1.61400000000000e-05
        258.150000000000	1.64000000000000e-05
        263.150000000000	1.66500000000000e-05
        268.150000000000	1.69000000000000e-05
        273.150000000000	1.71500000000000e-05
        278.150000000000	1.74000000000000e-05
        283.150000000000	1.76400000000000e-05
        288.150000000000	1.78900000000000e-05
        293.150000000000	1.81300000000000e-05
        298.150000000000	1.83700000000000e-05
        303.150000000000	1.86000000000000e-05
        308.150000000000	1.88350000000000e-05
        313.150000000000	1.90700000000000e-05
    ];

    % data check
    if z < 0
        z = 0;
    end

    % air temperature
    T = T0 - B*z;

    % air pressure
    p = p0*(T/T0)^(g0/(R*B));

    % air density
    rho = p/(R*T);

    % Legacy code: SUPER SLOW
    % % interpolation to find dynamic viscosity
    % mu = interp1(mu_data_old(:,1),mu_data_old(:,2),T);

    mu = DRSS.util.qinterp1(mu_data(:,1), mu_data(:,2), T, 1); % linear method
end
