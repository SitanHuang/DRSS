function w = wind_calc(wr,zr,z,xi)
% Wind

% ----------------------------------------------------------------------------
% Calculates the wind velocity at a specified altitude above the launch site.
% INPUT(S)= wr - reference (base) wind velocity [m/s]
%           zr - reference (base) altitude [m]
%           z - altitude above launch site [m]
%           xi - denominator of exponent in power law_
% OUTPUT(S)= w - wind speed [m/s]

    % calculates sustained wind speed
%     if z >=1219.2
%         w = - 11.83*cosd(360-303);
%     elseif z<1219.2 && z>=914.4
%         w = - (8.23 + 0.011811*(z - 914.4)) *cosd(360-297);
%     elseif z<914.4 && z>=609.6
%         w = - (6.69 + 0.0050525*(z - 609.6)) *cosd(360-292) ;
%     elseif z<609.6 && z>=304.8
%         w = - (5.66 + 0.00338*(z - 304.8)) *cosd(360-289);
%     elseif z<304.8
%         w = - 4.115*cosd(360-287);
%     end


    if z < 0
        w = wr;
        return;
    end
    w = wr*((z+zr)/zr)^(1/xi);
    % pause

end
