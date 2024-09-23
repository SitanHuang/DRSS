function [CD,CD_side,CL_side,S,S_side, tdone] = parachute_state(t_open,tf,Dp_f,CD_f,CD_side_f,CL_side_f,CD_type,style)

% Parachute State
% ----------------------------------------------------------------------------
% Calculates the drag coefficient, nominal (surface)
% area, and projected side area of a parachute as a function of time as the
% parachute opens (if the parachute is approximately fully open, the
% fully-opened values are returned)

% INPUT(S)= t_open - time since deployment [s]
%           tf - filling time (time to fully open) [s]
%           Dp_f - fully-open projected diameter [m]projected
%           CD_f - fully-open drag coefficient (nominal or ) [1]
%           CD_side_f - fully-open lateral drag coefficient [1]
%           CL_side_f - fully-open lateral lift coefficient [1]
%           CD_type - type of drag coefficient (nominal or projected)
%           style - 'f' for fourth order, 'e' for exponential

% OUTPUT(S)= CD - instantaneous drag coefficient (nominal or projected) [1]
%            CD_side - instantaneous sideslip drag coefficient [1]
%            CL_side - instantaneous sideslip lift coefficient [1]
%            S - instantaneous parachute area (nominal or projected) [m^2]
%            S_side - instantaneous sideslip area [m^2]

% drag coefficient, lift coefficient, nominal area, and projected
constep = 4;
tf1 = tf * constep;
    % diameter of parachute
if style == 'e'

    if t_open < tf * 12
        Dp = sqrt(Dp_f^2 * (1 - exp(- t_open / tf)));
        CD = 1 + (CD_f - 1)*(1 - exp(- t_open / tf));
        CD_side = 1 + (CD_side_f-1)*(1 - exp(- t_open / tf));
        CL_side = CL_side_f*(CD/CD_f);
        tdone = true;
    else
        tdone = false;
        Dp = Dp_f;
        CD = CD_f;
        CD_side = CD_side_f;
        CL_side = CL_side_f;
    end

elseif style == 'f'
     if t_open < tf1
        Dp = Dp_f*(t_open/(constep * tf));
        CD = 1 + (CD_f-1)*(t_open/(constep * tf))^2;
        CD_side = 1 + (CD_side_f-1)*(t_open/(constep * tf))^2;
        CL_side = CL_side_f*(CD/CD_f);
        tdone = true;
     else
        tdone = false;
        Dp = Dp_f;
        CD = CD_f;
        CD_side = CD_side_f;
        CL_side = CL_side_f;
     end
end

    % projected area
    Sp = pi*Dp^2/4;

    % nominal area
    So = Sp*2;

    % projected side area of parachute
    S_side = Sp/2;

    % selects which area to return
    if CD_type(1) == 'n'
        S = So;
    else
        S = Sp;
    end
end
