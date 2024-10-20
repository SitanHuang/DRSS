function [CA,CN,CD,CL,CP] = aerodynamics(Re,v,alpha,cm,D,L,L_nose,L_body,A_planform,l_tail,L_tail_c,L_tail_f,D_tail,A_fin,n_fins,A_fin_e,l_fin,t_fin,s,cr,ct,M_inf,A,CD_override,CDr)
%% aerodynamics.m
% Calculates the aerodynamic coefficients and CP on the rocket for given
% geometries/flight conditions.

% see http://cambridgerocket.sourceforge.net/AerodynamicCoefficients.pdf
% for where these equations are coming from - MJM 10/17/2022
% I will be commenting the number of the equation from the
% paper with (Eqn #)

% INPUT(S)= Re - Reynolds number at given altitude []
%           v - magnitude of rocket velocity [m/s]
%           alpha - angle of attack [rad]
%           cm - fin midchord length [m]
%           D - rocket diameter [m]
%           L - entire rocket length [m]
%           L_nose - nose cone length [m]
%           L_body - rocket body total length [m]
%           A_planform - rocket body projected side area not including fins [m^2]
%           l_tail - length to boat tail [m]
%           L_tail_c - boat tail curved section length [m]
%           L_tail_f - boat tail flat section length [m]
%           D_tail - boat tail aft diameter [m]
%           A_fin - fin area [m^2]
%           n_fins - number of fins []
%           A_fin_e - extended fin area [m^2]
%           l_fin - length to tip of fins from tip of nose [m]
%           t_fin - fin thickness [m]
%           s - fin semispan [m]
%           cr - fin root chord length [m]
%           ct - fin tip chord length [m]
%           M_inf - free stream Mach number
%           A - rocket cross-sectional area [m^2]
%           CD_override - boolean CD override
%           CDr - reference drag coefficient [1]

% OUTPUT(S)= CA - axial coefficient
%            CN - normal coefficient
%            CD - drag coefficient
%            CL - lift coefficient
%            CP - center of pressure [m]

    Re_c = 5e5; % critical reynolds number for transition

    % we assume the max. angle of attack for these equations is 10 deg
    if alpha > 0.174533
        alpha = 0.174533;
        %fprintf("Warning: alpha exceeded max in aerodynamics.m\n");
    elseif alpha < -0.174533
        alpha = -0.174533;
        %fprintf("Warning: alpha exceeded min for aerodynamics.m\n");
    end

    % ensure the Reynolds is not zero or below zero
%     if Re == 0
%         Re = .00001; % make arbitrarily small
%         fprintf("Warning: Re = 0 sent to aerodynamics.m\n");
%     end
%     if Re < 0
%         Re = .00001; % make arbitrarily small
%         fprintf("Warning: negative Reynolds number sent to aerodynamics.m\n");
%     end

    % nose cone
    CNa_nose = 2; % (Eqn 25)
    lcp_nose = .466 * L_nose; % (Eqn 27)

    % body tube (rocket body lift correction applied here)
    K = 1.5; % between 1 and 1.5 ??
    CNa_body = K * (A_planform / A) * alpha; % (Eqn 35)
    lcp_body = 0.5*L_body + L_nose; % assumed that nose and tail cancel out centroidaly, so half of body length is taken as location of body lift

    % fins
    Kfb = 1 + (D / (D + (2 * s))); % (Eqn 32)
    CNa_fin = Kfb * 4 * n_fins * (s / D)^2 / (1 + sqrt(1 + (2 * s/(cr + ct))^2)); % (Eqn 31)
    lcp_fin = l_fin + (cm * (cr + 2*ct))/(3*(cr + ct)) + (1 / 6) * (cr + ct - (cr*ct/(cr + ct))); % (Eqn 33)

    % boat tail
    CNa_tail = 2 * ((D_tail / D)^2 - 1); % (Eqn 29)
    lcp_tail = L_nose + L_body + ((L_tail_c / 3) * (1 + ((1 - (D / D_tail)) / (1 - ((D / D_tail)^2))))); % (Eqn 30)

    % normal coefficient
    CNa = CNa_nose + CNa_body + CNa_fin + CNa_tail; % (Eqn 23)
    CN = CNa * alpha / sqrt(1 - M_inf^2); % (Eqn 22 & 55)

    % center of pressure
    CP = (CNa_nose * lcp_nose + CNa_body * lcp_body + CNa_fin * lcp_fin + CNa_tail * lcp_tail) / CNa; % CN on body so small it can be neglected under 10 degrees % after introducing AAC, it appears larger alpha values are reached, so Body lift is no longer being neglected

    % core viscous friction coefficient
    if Re <= Re_c
        Cf_core = 1.328 / sqrt(Re); % (Eqn 45)
    else
        Cf_core = 0.074 / Re^(1/5) - (Re_c / Re) * (0.074 / (Re^(1/5)) - 1.328 / (sqrt(Re))); % (Eqn 45)
    end

    % fins viscous friction coefficient
    Re_fin = Re / L * cm; % change the characteristic length for the fin Reynolds
    if Re_fin <= Re_c
        Cf_fin = 1.328 / sqrt(Re_fin);
    else
        Cf_fin = 0.074 / Re_fin^(1/5) - (Re_c / Re_fin) * (0.074 / (Re_fin^(1/5)) - 1.328 / (sqrt(Re_fin)));
    end

    % different drag components at zero angle of attack
    CD_fb = ((1 + 60 / ((L / D)^3) + L_body / (400 * D))) * ((2.7 * L_nose / D) + (4 * L_body / D) + 2 * (1 - (D_tail / D)) * (L_tail_c / D)) * Cf_core; % (Eqn 41) rocket forebody drag
    CD_base = 0.029 * ((D_tail/D)^3) / sqrt(CD_fb); % (Eqn 42) base drag
    CD_fin = 2 * Cf_fin * ( 1 + 2 * t_fin / cm) * 4 * n_fins * A_fin / (pi * D^2); % (Eqn 43) fin drag
    CD_i = 2 * Cf_fin * ( 1 + 2 * t_fin / cm) * 4 * n_fins * (A_fin_e - A_fin) / ( pi * D^2); % (Eqn 44) interference drag

    % total zero angle of attack drag
    CD0 = CD_fb + CD_fin + CD_i + CD_base;

    % rocket body drag due to angle of attack
%     delta = 1 - 0.5 * exp(-0.2*abs(rad2deg(alpha))); % (Fig 4, graphical fit from plot)
%     eta = 1 - 0.5 * exp(-0.04*abs(rad2deg(alpha))); % (Fig 4, graphical fit from plot)
    delta = 0.716 * (alpha) + 0.73;
    eta = 0.573 * (alpha) + 0.56;
    CD_b_a = 2 * delta * alpha^2 + (3.6 * eta * (alpha^3) * (1.36 * L - 0.55 * L_nose)) / (pi * D^2); % (Eqn 49)

    % fin drag due to angle of attack
    R_fin = (2 * s + D) / D; % (Eqn 52.5)
    k_fb = 0.8065 * R_fin^2 + 1.553 * R_fin; % (Eqn 51)
    k_bf = 0.1935 * R_fin^2 + 0.8147 * R_fin + 1; % (Eqn 52)
    CD_f_a = alpha^2 * ((1.2 * A_fin_e * 4 / (pi * D^2)) + 3.12 * (k_fb + k_bf - 1) * A_fin * 4 / (pi * D^2)); % (Eqn 50)

    % total drag coefficient
    CD = (CD0 + CD_b_a + CD_f_a) / sqrt(1 - M_inf^2); % (Eqn 53 + 55)

    % override drag coefficient
    if CD_override
        % Pre Sitan Huang:
        % CD = CDr; % * (alpha * (.3 / .2967) + 1);

        % Sitan Huang's modifications:
        CD = CDr * (alpha * (.3 / .2967) + 1) / sqrt(1 - min(0.8, M_inf)^2);
        %fprintf("Warning: possibly inaccurate model used. Check CD \n");
    end

    % axial coefficient from geometry
    CA = (CD * cos(alpha) -  0.5 * CN * sin(2*alpha)) / (1 - sin(alpha)^2);

    % lift coefficient from geometry
    CL = (CN - CD * sin(alpha)) / cos(alpha); % CL is generally neglected using this model. It is calculated here but should never be used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
